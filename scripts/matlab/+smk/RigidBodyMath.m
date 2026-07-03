classdef RigidBodyMath
    %RIGIDBODYMATH  3D rigid-body transformation & rotation math primitives.
    %   All methods are static.  Rotation representations supported:
    %     - Elementary rotations about X, Y, Z axes
    %     - Axis-angle (Rodrigues formula)
    %     - Alignment struct (source axes → destination axes)
    %     - Rotation struct dispatcher (rpy / axis_angle / align / pending)

    methods (Static)

        %% construct a 4x4 homogeneous transformation matrix from rotation and translation
        function T = T(R, t)
            T = eye(4);
            T(1:3,1:3) = R;
            T(1:3,4) = t(:);
        end

        %% elementary rotation about the X axis
        function R = rotx(a)
            R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
        end

        %% elementary rotation about the Y axis
        function R = roty(a)
            R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
        end

        %% elementary rotation about the Z axis
        function R = rotz(a)
            R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
        end

        %% axis-angle → rotation matrix (Rodrigues formula)
        %   R = AXANG(AX, Q): R = I + sin(q)*K + (1-cos(q))*K^2
        function R = axang(ax, q)
            n = norm(ax);
            if n < eps
                R = eye(3);
                return;
            end
            w = ax(:) / n;
            K = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
            R = eye(3) + sin(q)*K + (1 - cos(q))*(K*K);
        end

        %% convert a string like 'X', '-Y', 'Z' into a 3x1 axis vector
        function v = axis_str(tok)
            s = 1;
            if tok(1) == '-'
                s = -1;
            end
            v = zeros(3,1);
            switch upper(tok(end))
                case 'X'; v(1) = s;
                case 'Y'; v(2) = s;
                case 'Z'; v(3) = s;
            end
        end

        %% evaluate an alignment struct into a numeric 3x3 rotation matrix
        %   s = source axis from child frame, d = destination axis in parent frame
        function R = align(al)
            s0 = smk.RigidBodyMath.axis_str(al.a{1});
            d0 = smk.RigidBodyMath.axis_str(al.a{2});
            s1 = smk.RigidBodyMath.axis_str(al.b{1});
            d1 = smk.RigidBodyMath.axis_str(al.b{2});
            SRC = [s0 s1 cross(s0, s1)];
            DST = [d0 d1 cross(d0, d1)];
            R = DST * SRC.';   % SRC orthonormal => inv = transpose
        end

        %% evaluate a rotation struct into a numeric 3x3 rotation matrix
        %   [R, PENDING] = ROT(ROT_STRUCT, PARAMS)
        %   PENDING indicates whether the rotation is not yet frozen (identity
        %   rotation returned; caller should flag in magenta).
        function [R, pending] = rot(rot_struct, params)
            pending = false;
            if ~isstruct(rot_struct)
                R = eye(3);
                return;
            end
            if isfield(rot_struct, 'align')
                R = smk.RigidBodyMath.align(rot_struct.align);
            elseif isfield(rot_struct, 'pending')
                R = eye(3);
                pending = true;
            elseif isfield(rot_struct, 'rpy')
                r = rot_struct.rpy;
                rx = smk.CommonUtils.eval_scalar(r{1}, params);
                ry = smk.CommonUtils.eval_scalar(r{2}, params);
                rz = smk.CommonUtils.eval_scalar(r{3}, params);
                R = smk.RigidBodyMath.rotz(rz) * smk.RigidBodyMath.roty(ry) * smk.RigidBodyMath.rotx(rx);
            elseif isfield(rot_struct, 'axis_angle')
                om = smk.CommonUtils.eval_vec(rot_struct.axis_angle.omega, params);
                q  = smk.CommonUtils.eval_scalar(rot_struct.axis_angle.q, params);
                R = smk.RigidBodyMath.axang(om, q);
            else
                R = eye(3);
            end
        end

    end
end
