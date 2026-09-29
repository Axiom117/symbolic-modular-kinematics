classdef EdgeGraph < handle
%EDGEGRAPH  Shared pose-graph intermediate representation (IR).
%
%   ROLE | 类定位
%   -------------------------
%   A handle-class "Data Container" for the directed pose graph that 
%   accumulates pose-graph data (directed edges + root labels) and 
%   hands them to the +core/PosePropagator FK engine.
%
%   CONTAINED DATA | 包含的成员
%   ---------------------------
%   Edges     : struct array with fields {from, to, T, kind}
%               from/to : char — 帧 / 节点名
%               T       : 4x4 double homogeneous transform
%               kind    : char — 'fixed' | 'joint' | 'mate' | 'closed_mate'
%   RootNode  : char — frame name that seeds FK propagation (single root)
%
%   USAGE | 用法
%   --------------------------------
%       g = ir.EdgeGraph();
%       g.addFixedTransform('frame0.body','frame0.faceXPlus', T);
%       g.addJoint('j1.linkA','j1.linkB', [1;0;0], q, 'revolute');
%       g.addMateBidirectional('frame0.faceXPlus','j1.linkA', 0, 4);
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

        % RootNode – frame name that seeds FK propagation (single root)
        RootNode (1,:) char = ''
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
            obj.addEdge(to, from, core.RigidBodyMath.invT(T), 'fixed');
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
            obj.addEdge(to, from, core.RigidBodyMath.invT(T), 'joint');
        end

        %% addMateBidirectional  Insert a bidirectional mate edge pair (socket↔plug).
        %   obj.addMateBidirectional(SOCKET, PLUG, ROLL, SYMMETRY)
        %     SOCKET    : char — socket-frame node name
        %     PLUG      : char — plug-frame node name
        %     ROLL      : integer (default 0) — roll index (0..symmetry-1)
        %     SYMMETRY  : integer (default 4) — rotational symmetry count
        %   Mate transform: T = Rz(roll * 2*pi/symmetry) * Rx(pi)
        %   See specs/dsl/connection-semantics.md for the convention.
        function addMateBidirectional(obj, socket, plug, roll, symmetry)
            if nargin < 5 || isempty(symmetry); symmetry = 4; end
            if nargin < 4 || isempty(roll); roll = 0; end
            Tm = obj.mateTransform(roll, symmetry);

            % bidirectional edges: SOCKET→PLUG and PLUG→SOCKET (inverse transform), ensuring that the graph is traversable in either direction
            obj.addEdge(socket, plug, Tm, 'mate');
            obj.addEdge(plug, socket, core.RigidBodyMath.invT(Tm), 'mate');
        end

        %% addMateUnidirectional  Insert a one-directional diagnostic-only mate edge.
        %   Used for chord edges in closed kinematic loops.  These edges
        %   are NOT propagated through (they are the cut of a loop);
        %   they exist only to report the loop-closure residual gap.
        %   Unlike addMateBidirectional, this does NOT insert a reverse edge.
        function addMateUnidirectional(obj, socket, plug, roll, symmetry)
            if nargin < 5 || isempty(symmetry); symmetry = 4; end
            if nargin < 4 || isempty(roll); roll = 0; end
            Tm = obj.mateTransform(roll, symmetry);

            % one-way edge for loop-closure diagnostics; kind='closed_mate'
            % ensures exportEdges() excludes it from FK propagation
            obj.addEdge(socket, plug, Tm, 'closed_mate');
        end

        %% addRoot  Register the propagation root (seed pose = eye(4)).
        %   EdgeGraph supports a SINGLE root node.  Registering a second,
        %   different node raises an error; re-registering the same node
        %   is a no-op.
        %
        %   In the tool-rooted growth paradigm, the root is typically a
        %   tool reference frame (e.g. ToolPipette.tip_origin) from which
        %   the mechanism grows outward toward manipulator modules.
        function addRoot(obj, node)
            if isempty(obj.RootNode)
                obj.RootNode = node;
            elseif ~strcmp(obj.RootNode, node)
                error('ir:EdgeGraph:multipleRoots', ...
                    ['EdgeGraph supports a single root node. ' ...
                     'Already registered "%s", got "%s".'], ...
                    obj.RootNode, node);
            end
        end

        %% propagate  Run FK propagation and return a pose map.
        %   poses = g.propagate()
        %     returns containers.Map where keys are frame names and
        %     values are 4x4 homogeneous transforms.
        %     If no root node is registered, the 'from' field of the
        %     first edge is used as the root.
        function poses = propagate(obj)
            seed = containers.Map('KeyType', 'char', 'ValueType', 'any');
            if ~isempty(obj.RootNode)
                seed(obj.RootNode) = eye(4);
            elseif ~isempty(obj.Edges)
                % use the 'from' node of the first edge as the root if no root node is registered
                seed(obj.Edges(1).from) = eye(4);
            end
            edgeStruct = obj.exportEdges();
            poses = core.PosePropagator.propagatePoses(edgeStruct, seed);
        end

        %% exportEdges  Export the FK-ready edge array (Edges minus chord edges).
        %   s = g.exportEdges() returns a struct array with fields
        %   'from', 'to', 'T' — exactly the format consumed by
        %   PosePropagator.propagatePoses(edges, seed).  Closed-mate
        %   (chord) edges are excluded and 'kind' metadata is dropped.
        function s = exportEdges(obj)
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

        %% numRootNodes  Number of registered root nodes (0 or 1).
        function n = numRootNodes(obj)
            n = double(~isempty(obj.RootNode));
        end

        %% hasRootNodes  True when a root node is registered.
        function tf = hasRootNodes(obj)
            tf = ~isempty(obj.RootNode);
        end

    end

    % ---- private helpers ----
    methods (Access = private)

        %% mateTransform  Build the socket→plug mate transform (shared helper).
        %   Tm = obj.mateTransform(ROLL, SYMMETRY)
        %     returns the 4x4 homogeneous transform
        %     T = Rz(roll * 2*pi/symmetry) * Rx(pi), t = 0.
        %   Used by both addMateBidirectional and addMateUnidirectional.
        %   See specs/dsl/connection-semantics.md for the convention.
        function Tm = mateTransform(~, roll, symmetry)
            rollAngle = roll * 2 * pi / symmetry;
            Tm = core.RigidBodyMath.T( ...
                core.RigidBodyMath.rotz(rollAngle) * core.RigidBodyMath.rotx(pi), ...
                [0; 0; 0]);
        end

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
