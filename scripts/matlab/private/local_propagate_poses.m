%% propagate global poses through an edge graph via iterative forward kinematics
%   POSES = LOCAL_PROPAGATE_POSES(EDGES, ROOTNAME, ROOTPOSE) seeds a
%   containers.Map with ROOTNAME -> ROOTPOSE (4x4) and repeatedly walks
%   EDGES, computing poses(e.to) = poses(e.from) * e.T whenever e.from is
%   already placed and e.to is not. This handles any DAG and gracefully
%   stops on closed-loop graphs: a chord edge whose 'to' frame is already
%   placed (via the tree path) is skipped, so the tree-path pose wins.
%
%   EDGES is a struct array with at least fields 'from', 'to', 'T'.
%   ROOTNAME is a char frame name; ROOTPOSE is its 4x4 world pose.
%
%   POSES is a containers.Map from frame name (char) to 4x4 pose.
%
%   Multiple roots: call with the primary root, then the returned map can be
%   re-seeded and passed back in via LOCAL_PROPAGATE_POSES(EDGES, POSES) to
%   continue from an existing (already-seeded) map.
function poses = local_propagate_poses(edges, rootArg, rootPose)
    if isa(rootArg, 'containers.Map')
        % continuation form: caller supplies a pre-seeded pose map
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
