classdef PosePropagator
    %POSEPROPAGATOR  Joint transforms & iterative FK pose propagation.
    %   All methods are static.  Handles revolute/prismatic joint edge
    %   construction and forward-kinematics propagation through a frame
    %   DAG (handles multi-root and loop graphs gracefully).

    methods (Static)

        %% build a joint edge transform for a given kind, axis and variable value
        %   T = JOINT_TRANSFORM(KIND, AX, VAL)
        %     - revolute (default): pure rotation VAL (rad) about axis AX
        %     - prismatic:          pure translation VAL (mm) along axis AX
        function T = jointTransform(kind, ax, val)
            switch lower(kind)
                case 'prismatic'
                    n = norm(ax);
                    if n < eps
                        d = [0; 0; 0];
                    else
                        d = ax(:) / n * val;
                    end
                    T = core.RigidBodyMath.T(eye(3), d);
                otherwise  % revolute
                    T = core.RigidBodyMath.T(core.RigidBodyMath.axang(ax, val), [0; 0; 0]);
            end
        end

        %% Iterative FK that propagates global poses through an edge graph
        % use iterative propagation to compute the global pose of each frame in a DAG of edges, starting from one or more known root frames
        %   POSES = PROPAGATE_POSES(EDGES, ROOTNAME, ROOTPOSE)
        %     seeds a containers.Map with ROOTNAME -> ROOTPOSE (4x4) and
        %     repeatedly walks EDGES, computing poses(e.to) = poses(e.from)*e.T.
        %
        %   POSES = PROPAGATE_POSES(EDGES, SEEDMAP)
        %     continuation form: caller supplies a pre-seeded pose map.
        function poses = propagatePoses(edges, rootArg, rootPose)
            % if rootArg is a string, seed the pose map with rootPose; otherwise assume it's a pre-seeded containers.Map
            if isa(rootArg, 'containers.Map')
                poses = rootArg;
            else
                poses = containers.Map('KeyType', 'char', 'ValueType', 'any');
                % seed the root frame pose
                poses(rootArg) = rootPose;
            end
            changed = true;

            % each iteration, propagate poses through the edges from the beginning; repeat until no new poses are computed
            while changed
                changed = false;
                for k = 1:numel(edges)
                    e = edges(k);

                    % if the "from" frame is known and the "to" frame is not, propagate the pose
                    if isKey(poses, e.from) && ~isKey(poses, e.to)
                        % compute the "to" frame pose from the "from" frame pose and the edge transform
                        poses(e.to) = poses(e.from) * e.T;
                        changed = true;
                    end
                end
            end
        end

    end
end
