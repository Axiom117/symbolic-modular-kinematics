classdef EdgeGraph < handle
%EDGEGRAPH  Shared pose-graph intermediate representation (IR).
%   A handle-class container for the directed pose graph that sits
%   between module/mechanism YAML parsing and FK propagation.
%
%   EDGEGRAPH is NOT a solver — it accumulates edges (fixed-transforms,
%   joints, mates) and ground-node labels, then feeds them to the
%   existing +core/PoseGraph propagation engine.
%
%   Why a handle class:
%     - viz.module and viz.mechanism build edges incrementally across
%       many sub-function calls.  MATLAB value semantics would force
%       pass-in/pass-out of the accumulator on every call.
%     - A handle class provides in-place mutation, keeping call sites
%       clean while retaining the existing struct-based edge format
%       that PoseGraph.propagatePoses expects.
%
%   Usage (mechanism context):
%       g = ir.EdgeGraph();
%       g.addFixedTransform('frame0.body','frame0.faceX+', T, false);
%       g.addJoint('j1.linkA','j1.linkB', [1;0;0], q, 'revolute');
%       g.addMate('frame0.faceX+','j1.linkA', 0, 4);
%       g.addGround('manipulator.ground');
%       poses = g.propagate();
%
%   See also: +core/PoseGraph, +viz/mechanism, +viz/module

    % ---- public properties ----
    properties (SetAccess = private)
        % Edges  – struct array with fields: from, to, T, kind, pending
        %   from/to  : char (frame / node name)
        %   T        : 4x4 double homogeneous transform
        %   kind     : char — 'fixed' | 'joint' | 'mate'
        %   pending  : logical — true when rotation is not yet resolved
        Edges (:,1) struct = struct('from', {}, 'to', {}, 'T', {}, 'kind', {}, 'pending', {})

        % GroundNodes – cell array of frame names bound to world origin
        GroundNodes (:,1) cell = {}
    end

    % ---- public methods ----
    methods

        %% addFixedTransform  Insert a bidirectional fixed-transform edge pair.
        %   obj.addFixedTransform(FROM, TO, T, ISPENDING)
        %     FROM, TO  : char — frame / node names
        %     T         : 4x4 double homogeneous transform (FROM→TO)
        %     ISPENDING : logical (default false) — true if rotation
        %                 component is a placeholder (an align rule with
        %                 unresolved parameters).
        function addFixedTransform(obj, from, to, T, isPending)
            if nargin < 5; isPending = false; end

            % bidirectional edges: FROM→TO and TO→FROM (inverse transform), ensuring that the graph is traversable in either direction
            obj.addEdge(from, to, T, 'fixed', isPending);
            obj.addEdge(to, from, localInvT(T), 'fixed', isPending);
        end

        %% addJoint  Insert a bidirectional joint edge pair.
        %   obj.addJoint(FROM, TO, AXIS, VALUE, KIND)
        %     KIND  : 'revolute' (default) or 'prismatic'
        %     AXIS  : 3x1 numeric — joint axis direction in parent frame
        %     VALUE : scalar — angle (rad) for revolute, displacement
        %             (mm) for prismatic.
        function addJoint(obj, from, to, axis, value, kind)
            if nargin < 6 || isempty(kind); kind = 'revolute'; end
            T = core.PoseGraph.jointTransform(kind, axis, value);
            obj.addEdge(from, to, T, 'joint', false);
            obj.addEdge(to, from, localInvT(T), 'joint', false);
        end

        %% addMate  Insert a bidirectional mate edge pair (socket↔plug).
        %   obj.addMate(SOCKET, PLUG, ROLL, SYMMETRY)
        %     SOCKET    : char — socket-frame node name
        %     PLUG      : char — plug-frame node name
        %     ROLL      : integer (default 0) — roll index (0..symmetry-1)
        %     SYMMETRY  : integer (default 4) — rotational symmetry count
        %   Mate transform: T = Rz(roll * 2*pi/symmetry) * Rx(pi)
        %   See specs/dsl/connection-semantics.md for the convention.
        function addMate(obj, socket, plug, roll, symmetry)
            if nargin < 5 || isempty(symmetry); symmetry = 4; end
            if nargin < 4 || isempty(roll); roll = 0; end
            rollAngle = roll * 2 * pi / symmetry;
            Tm = core.RigidBodyMath.T( ...
                core.RigidBodyMath.rotz(rollAngle) * core.RigidBodyMath.rotx(pi), ...
                [0; 0; 0]);
            obj.addEdge(socket, plug, Tm, 'mate', false);
            obj.addEdge(plug, socket, localInvT(Tm), 'mate', false);
        end

        %% addClosedMate  Insert a one-directional diagnostic-only mate edge.
        %   Used for chord edges in closed kinematic loops.  These edges
        %   are NOT propagated through (they are the cut of a loop);
        %   they exist only to report the loop-closure residual gap.
        %   Unlike addMate, this does NOT insert a reverse edge.
        function addClosedMate(obj, socket, plug, roll, symmetry)
            if nargin < 5 || isempty(symmetry); symmetry = 4; end
            if nargin < 4 || isempty(roll); roll = 0; end
            rollAngle = roll * 2 * pi / symmetry;
            Tm = core.RigidBodyMath.T( ...
                core.RigidBodyMath.rotz(rollAngle) * core.RigidBodyMath.rotx(pi), ...
                [0; 0; 0]);
            obj.addEdge(socket, plug, Tm, 'mate', false);
        end

        %% addGround  Register a frame as bound to world origin.
        %   During propagate(), every ground node is seeded with
        %   pose = eye(4).  Multiple ground nodes are supported for
        %   multi-branch / parallel mechanisms.
        function addGround(obj, node)
            obj.GroundNodes{end+1} = node;
        end

        %% propagate  Run FK propagation and return a pose map.
        %   poses = g.propagate()
        %     returns containers.Map where keys are frame names and
        %     values are 4x4 homogeneous transforms.
        %     If no ground nodes are registered, the 'from' field of the
        %     first edge is used as the root.
        function poses = propagate(obj)
            seed = containers.Map('KeyType', 'char', 'ValueType', 'any');
            if ~isempty(obj.GroundNodes)
                for k = 1:numel(obj.GroundNodes)
                    seed(obj.GroundNodes{k}) = eye(4);
                end
            elseif ~isempty(obj.Edges)
                seed(obj.Edges(1).from) = eye(4);
            end
            edgeStruct = obj.toStruct();
            poses = core.PoseGraph.propagatePoses(edgeStruct, seed);
        end

        %% toStruct  Export edges as the struct array that PoseGraph expects.
        %   s = g.toStruct() returns a struct array with fields
        %   'from', 'to', 'T' — exactly the format consumed by
        %   PoseGraph.propagatePoses(edges, seed).
        function s = toStruct(obj)
            s = obj.Edges;
            % drop the 'kind' and 'pending' metadata fields that the
            % FK engine does not need
            s = rmfield(s, 'kind');
            s = rmfield(s, 'pending');
        end

        %% findMates  Return all mate / closed-mate edges for diagnostics.
        %   mates = g.findMates()
        %     returns a struct array (subset of Edges) with kind='mate'.
        function mates = findMates(obj)
            if isempty(obj.Edges)
                mates = struct('from', {}, 'to', {}, 'T', {}, 'kind', {}, 'pending', {});
                return;
            end
            mateMask = strcmp({obj.Edges.kind}, 'mate');
            mates = obj.Edges(mateMask);
        end

        %% countByKind  Count edges of each kind.
        %   c = g.countByKind() returns a struct with fields
        %   'fixed', 'joint', 'mate'.
        function c = countByKind(obj)
            c.fixed = 0; c.joint = 0; c.mate = 0;
            if isempty(obj.Edges); return; end
            kinds = {obj.Edges.kind};
            c.fixed = nnz(strcmp(kinds, 'fixed'));
            c.joint = nnz(strcmp(kinds, 'joint'));
            c.mate  = nnz(strcmp(kinds, 'mate'));
        end

        %% numEdges  Total number of directed edges.
        function n = numEdges(obj)
            n = numel(obj.Edges);
        end

        %% numGroundNodes  Number of registered ground nodes.
        function n = numGroundNodes(obj)
            n = numel(obj.GroundNodes);
        end

    end

    % ---- private helpers ----
    methods (Access = private)

        %% addEdge  Append a single directed edge (internal).
        function addEdge(obj, from, to, T, kind, isPending)
            obj.Edges(end+1) = struct( ...
                'from',    from, ...
                'to',      to, ...
                'T',       T, ...
                'kind',    kind, ...
                'pending', isPending);
        end

    end

end

%% ---- local function (not a method) ----

function Ti = localInvT(T)
    R = T(1:3,1:3); t = T(1:3,4);
    Ti = eye(4); Ti(1:3,1:3) = R'; Ti(1:3,4) = -R' * t;
end
