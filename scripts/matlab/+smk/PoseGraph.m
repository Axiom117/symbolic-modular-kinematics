classdef PoseGraph
    %POSEGRAPH  Joint transforms & iterative FK pose propagation.
    %   All methods are static.  Handles revolute/prismatic joint edge
    %   construction and forward-kinematics propagation through a frame
    %   DAG (handles multi-root and loop graphs gracefully).

    methods (Static)

        %% build a joint edge transform for a given kind, axis and variable value
        %   T = JOINT_TRANSFORM(KIND, AX, VAL)
        %     - revolute (default): pure rotation VAL (rad) about axis AX
        %     - prismatic:          pure translation VAL (mm) along axis AX
        function T = joint_transform(kind, ax, val)
            if nargin < 1 || isempty(kind)
                kind = 'revolute';
            end
            switch lower(kind)
                case 'prismatic'
                    n = norm(ax);
                    if n < eps
                        d = [0; 0; 0];
                    else
                        d = ax(:) / n * val;
                    end
                    T = smk.RigidBodyMath.T(eye(3), d);
                otherwise  % revolute
                    T = smk.RigidBodyMath.T(smk.RigidBodyMath.axang(ax, val), [0; 0; 0]);
            end
        end

        %% propagate global poses through an edge graph via iterative FK
        %   POSES = PROPAGATE_POSES(EDGES, ROOTNAME, ROOTPOSE)
        %     seeds a containers.Map with ROOTNAME -> ROOTPOSE (4x4) and
        %     repeatedly walks EDGES, computing poses(e.to) = poses(e.from)*e.T.
        %
        %   POSES = PROPAGATE_POSES(EDGES, SEEDMAP)
        %     continuation form: caller supplies a pre-seeded pose map.
        function poses = propagate_poses(edges, rootArg, rootPose)
            if isa(rootArg, 'containers.Map')
                poses = rootArg;
            else
                poses = containers.Map('KeyType', 'char', 'ValueType', 'any');
                poses(rootArg) = rootPose;
            end
            changed = true;
            while changed
                changed = false;
                for k = 1:numel(edges)
                    e = edges(k);
                    if isKey(poses, e.from) && ~isKey(poses, e.to)
                        poses(e.to) = poses(e.from) * e.T;
                        changed = true;
                    end
                end
            end
        end

        %% check if a frame is pending (not yet frozen) based on the edges
        function pend = frame_pending(edges, name)
            pend = false;
            for k = 1:numel(edges)
                if strcmp(edges(k).to, name) && edges(k).pending
                    pend = true;
                    return;
                end
            end
        end

    end
end
