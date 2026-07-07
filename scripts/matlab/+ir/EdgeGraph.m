classdef EdgeGraph < handle
%EDGEGRAPH  Shared pose-graph intermediate representation (IR).
%   A handle-class container for the directed pose graph that sits
%   between module/mechanism YAML parsing and FK propagation.
%
%   EDGEGRAPH is NOT a solver — it accumulates edges (fixed-transforms,
%   joints, mates) and ground-node labels, then feeds them to the
%   existing +core/PosePropagator propagation engine.
%
%   Why a handle class:
%     - viz.module and viz.mechanism build edges incrementally across
%       many sub-function calls.  MATLAB value semantics would force
%       pass-in/pass-out of the accumulator on every call.
%     - A handle class provides in-place mutation, keeping call sites
%       clean while retaining the existing struct-based edge format
%       that PosePropagator.propagatePoses expects.
%
%   Usage (mechanism context):
%       g = ir.EdgeGraph();
%       g.addFixedTransform('frame0.body','frame0.faceXPlus', T);
%       g.addJoint('j1.linkA','j1.linkB', [1;0;0], q, 'revolute');
%       g.addMate('frame0.faceXPlus','j1.linkA', 0, 4);
%       g.addRoot('toolpipette.tip_origin');
%       poses = g.propagate();
%
%   See also: +core/PosePropagator, +viz/mechanism, +viz/module

    % ---- public properties ----
    properties (SetAccess = private)
        % Edges  – struct array with fields: from, to, T, kind
        %   from/to  : char (frame / node name)
        %   T        : 4x4 double homogeneous transform
        %   kind     : char — 'fixed' | 'joint' | 'mate'
        Edges (:,1) struct = struct('from', {}, 'to', {}, 'T', {}, 'kind', {})

        % RootNodes – cell array of frame names that seed FK propagation
        RootNodes (:,1) cell = {}
    end

    % ---- public methods ----
    methods

        %% addFixedTransform  Insert a bidirectional fixed-transform edge pair.
        %   obj.addFixedTransform(FROM, TO, T)
        %     FROM, TO  : char — frame / node names
        %     T         : 4x4 double homogeneous transform (FROM→TO)
        function addFixedTransform(obj, from, to, T)

            % bidirectional edges: FROM→TO and TO→FROM (inverse transform), ensuring that the graph is traversable in either direction
            obj.addEdge(from, to, T, 'fixed');
            obj.addEdge(to, from, localInvT(T), 'fixed');
        end

        %% addJoint  Insert a bidirectional joint edge pair.
        %   obj.addJoint(FROM, TO, AXIS, VALUE, KIND)
        %     KIND  : 'revolute' (default) or 'prismatic'
        %     AXIS  : 3x1 numeric — joint axis direction in parent frame
        %     VALUE : scalar — angle (rad) for revolute, displacement
        %             (mm) for prismatic.
        function addJoint(obj, from, to, axis, value, kind)
            T = core.PosePropagator.jointTransform(kind, axis, value);
            obj.addEdge(from, to, T, 'joint');
            obj.addEdge(to, from, localInvT(T), 'joint');
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

            % bidirectional edges: SOCKET→PLUG and PLUG→SOCKET (inverse transform), ensuring that the graph is traversable in either direction
            obj.addEdge(socket, plug, Tm, 'mate');
            obj.addEdge(plug, socket, localInvT(Tm), 'mate');
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

            % one-way edge for loop-closure diagnostics; kind='closed_mate'
            % ensures toStruct() excludes it from FK propagation
            obj.addEdge(socket, plug, Tm, 'closed_mate');
        end

        %% addRoot  Register a frame as a propagation root (seed pose = eye(4)).
        %   During propagate(), every root node is seeded with
        %   pose = eye(4).  Multiple root nodes are supported for
        %   multi-branch / parallel mechanisms.
        %
        %   In the tool-rooted growth paradigm, the root is typically a
        %   tool reference frame (e.g. ToolPipette.tip_origin) from which
        %   the mechanism grows outward toward manipulator modules.
        function addRoot(obj, node)
            obj.RootNodes{end+1} = node;
        end

        %% propagate  Run FK propagation and return a pose map.
        %   poses = g.propagate()
        %     returns containers.Map where keys are frame names and
        %     values are 4x4 homogeneous transforms.
        %     If no root nodes are registered, the 'from' field of the
        %     first edge is used as the root.
        function poses = propagate(obj)
            seed = containers.Map('KeyType', 'char', 'ValueType', 'any');
            if ~isempty(obj.RootNodes)
                for k = 1:numel(obj.RootNodes)
                    seed(obj.RootNodes{k}) = eye(4);
                end
            elseif ~isempty(obj.Edges)
                % use the 'from' node of the first edge as the root if no root nodes are registered
                seed(obj.Edges(1).from) = eye(4);
            end
            edgeStruct = obj.toStruct();
            poses = core.PosePropagator.propagatePoses(edgeStruct, seed);
        end

        %% toStruct  Export edges as the struct array that PosePropagator expects.
        %   s = g.toStruct() returns a struct array with fields
        %   'from', 'to', 'T' — exactly the format consumed by
        %   PosePropagator.propagatePoses(edges, seed).
        function s = toStruct(obj)
            % exclude closed_mate (diagnostic-only) edges from FK propagation.
            % closed_mate edges represent chord cuts of kinematic loops and
            % must not participate in pose propagation — their sole purpose
            % is to report loop-closure residuals (gap / Zdot) after FK.
            keepMask = ~strcmp({obj.Edges.kind}, 'closed_mate');
            s = obj.Edges(keepMask);
            % drop the 'kind' metadata field that the FK engine does not need
            s = rmfield(s, 'kind');
        end

        %% findMates  Return all mate / closed-mate edges for diagnostics.
        %   mates = g.findMates()
        %     returns a struct array (subset of Edges) with kind='mate'.
        function mates = findMates(obj)
            if isempty(obj.Edges)
                mates = struct('from', {}, 'to', {}, 'T', {}, 'kind', {});
                return;
            end
            mateMask = strcmp({obj.Edges.kind}, 'mate') | strcmp({obj.Edges.kind}, 'closed_mate');
            mates = obj.Edges(mateMask);
        end

        %% countByKind  Count edges of each kind.
        %   c = g.countByKind() returns a struct with fields
        %   'fixed', 'joint', 'mate'.
        function c = countByKind(obj)
            c.fixed = 0; c.joint = 0; c.mate = 0; c.closed_mate = 0;
            if isempty(obj.Edges); return; end
            kinds = {obj.Edges.kind};
            c.fixed = nnz(strcmp(kinds, 'fixed'));
            c.joint = nnz(strcmp(kinds, 'joint'));
            c.mate  = nnz(strcmp(kinds, 'mate'));
            c.closed_mate = nnz(strcmp(kinds, 'closed_mate'));
        end

        %% numEdges  Total number of directed edges.
        function n = numEdges(obj)
            n = numel(obj.Edges);
        end

        %% numRootNodes  Number of registered root nodes.
        function n = numRootNodes(obj)
            n = numel(obj.RootNodes);
        end

        %% hasRootNodes  True when at least one root node is registered.
        function tf = hasRootNodes(obj)
            tf = ~isempty(obj.RootNodes);
        end

    end

    % ---- private helpers ----
    methods (Access = private)

        %% addEdge  Append a single directed edge (internal).
        function addEdge(obj, from, to, T, kind)
            obj.Edges(end+1) = struct( ...
                'from',    from, ...
                'to',      to, ...
                'T',       T, ...
                'kind',    kind);
        end

    end

end

%% ---- local function (not a method) ----

function Ti = localInvT(T)
    R = T(1:3,1:3); t = T(1:3,4);
    if isa(T, 'sym')
        Ti = sym(eye(4));
    else
        Ti = eye(4);
    end
    Ti(1:3,1:3) = R';
    Ti(1:3,4) = -R' * t;
end
