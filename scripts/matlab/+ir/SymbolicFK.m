classdef SymbolicFK < handle
%SYMBOLICFK  Symbolic forward-kinematics engine (thin wrapper over EdgeGraph).
%   Extracts the symbolic 4×4 homogeneous pose of a named end frame from a
%   symbolically-expanded EdgeGraph (built by ir.Expander in symbolicMode=true).
%
%   SYMBOLICFK is not a solver — it queries the already-propagated pose map
%   from EdgeGraph and decomposes the result into position and rotation
%   symbolic expressions.  The underlying FK propagation is handled by
%   EdgeGraph.propagate() → PoseGraph.propagatePoses(), which already
%   supports mixed double/sym transform matrices via MATLAB's polymorphic
%   arithmetic operators.
%
%   Usage:
%       e = ir.Expander(dslYaml, '', true);            % symbolic mode
%       fk = ir.SymbolicFK(e.EdgeGraph_, 'pipette.tip_origin');
%       % fk.TSym     — 4×4 sym  homogeneous transform (world→endFrame)
%       % fk.PosExpr  — 3×1 sym  position [x; y; z]
%       % fk.RotExpr  — 3×3 sym  rotation matrix
%       % fk.JointVars — sym array of all joint variables on the path
%
%       % Evaluate at specific joint values:
%       T_num = double(subs(fk.TSym, fk.JointVars, [0.5236; -0.7854]));
%
%   See also: +ir/EdgeGraph, +ir/Expander, +core/PoseGraph

    % ---- public read-only properties ----
    properties (SetAccess = private)
        TSym          % 4×4 sym — world→endFrame homogeneous transform
        PosExpr       % 3×1 sym — position component of TSym
        RotExpr       % 3×3 sym — rotation component of TSym
        JointVars     % sym array — all symbolic joint variables appearing in TSym
        EndFrame      (1,:) char  % name of the target frame
    end

    % ---- public methods ----
    methods

        %% Constructor: propagate FK and extract the end-frame symbolic pose.
        %   obj = SymbolicFK(EDGEGRAPH, ENDFRAME)
        %     EDGEGRAPH : ir.EdgeGraph with symbolic joint edges (built by
        %                 Expander in symbolicMode=true).  Fixed and mate
        %                 edges may be numeric (double) — they are
        %                 auto-promoted during propagation.
        %     ENDFRAME  : char — fully-qualified frame name (e.g.
        %                 'pipette.tip_origin') reachable from ground nodes
        %                 via spanning-tree edges.
        function obj = SymbolicFK(edgeGraph, endFrame)
            arguments
                edgeGraph  (1,1) ir.EdgeGraph
                endFrame   (1,:) char
            end

            obj.EndFrame = endFrame;

            % propagate poses through the symbolic edge graph.
            % PoseGraph.propagatePoses handles mixed double/sym T matrices
            % transparently; closed_mate edges are excluded by toStruct().
            poses = edgeGraph.propagate();

            assert(isKey(poses, endFrame), ...
                'ir:SymbolicFK:endFrameNotFound', ...
                ['End frame "%s" not found in propagated poses. ' ...
                 'Check that the frame is reachable from a ground node.'], ...
                endFrame);

            % extract the end-frame pose and decompose
            obj.TSym = poses(endFrame);
            obj.PosExpr = obj.TSym(1:3, 4);
            obj.RotExpr = obj.TSym(1:3, 1:3);

            % auto-detect all symbolic variables on the FK path.
            % symvar returns them sorted alphabetically by name.
            obj.JointVars = symvar(obj.TSym);
        end

        %% Evaluate the symbolic pose at specific joint values (numeric).
        %   T = EVAL(obj, VALS)
        %     VALS : numeric vector — joint values in the same order as
        %            obj.JointVars (use symvar order: alphabetical by name).
        %     T    : 4×4 double — numeric homogeneous transform.
        function T = eval(obj, vals)
            arguments
                obj
                vals (:,1) double
            end
            assert(numel(vals) == numel(obj.JointVars), ...
                'ir:SymbolicFK:valCount', ...
                'Expected %d joint values, got %d.', ...
                numel(obj.JointVars), numel(vals));
            T = double(subs(obj.TSym, obj.JointVars, reshape(vals, size(obj.JointVars))));
        end

        %% Evaluate position only at specific joint values.
        function p = evalPos(obj, vals)
            T = obj.eval(vals);
            p = T(1:3, 4);
        end

        %% Evaluate rotation matrix only at specific joint values.
        function R = evalRot(obj, vals)
            T = obj.eval(vals);
            R = T(1:3, 1:3);
        end

    end

end
