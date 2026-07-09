classdef ClosureSolver < handle
%CLOSURESOLVER  Closed-loop constraint construction and numerical solving.
%   Builds symbolic loop-closure residuals from an expanded kinematic graph,
%   converts them to numeric function handles, and bridges to fmincon for
%   inverse kinematics solving.
%
%   Residual construction:
%     For each closure cut (near=socket, far=plug), the mate transform M
%     (Rz(roll*2π/sym)*Rx(π)) encodes the physical connection.  At closure,
%     T_far should equal T_near * M.  The residual is defined as:
%         T_res = (T_near * M) \ T_far   →   should be I at closure.
%     This is decomposed into a 6×1 pose error vector [tx,ty,tz,rx,ry,rz]
%     (Z-Y-X Euler angles for rotation).
%
%   Usage:
%       e   = ir.Expander(dslYaml);
%       cfg = ir.ExecutionConfig(execYaml, e.SymbolRegistry, e.EdgeGraph_);
%       cs  = solver.ClosureSolver(e.EdgeGraph_, cfg, e.JointVarMap);
%
%       % Zero-pose verification:
%       cs.verifyZeroPose();
%
%       % Evaluate at specific joint values:
%       r = cs.evalResidual([q_AB; q_BD; q_CD; q_AC]);
%
%       % Solve IK via fmincon:
%       [x_opt, fval] = cs.solveIK();
%
%   See also: +ir/EdgeGraph, +ir/ExecutionConfig, +solver/KinematicModel

    % ---- public read-only properties ----
    properties (SetAccess = private)
        ClosureCuts   (:,1) struct          % struct array: near, far, components, T_mate (4×4 double)
        UnknownSyms   (1,:) sym             % sym array: unknown joint variable handles
        UnknownNames  (1,:) cell            % cell array: canonical names of unknown variables
        ResidualSym   (:,1) sym             % sym column vector: full residual expression
        Tolerances    (1,1) struct          % struct: translation_mm, rotation_rad
        NumCuts       (1,1) double          % number of closure cuts
        NumComponents (1,1) double          % total number of residual components
    end

    % ---- private properties ----
    properties (Access = private)
        Poses_                             % containers.Map: frame name → 4×4 sym
        JointVarMap_                       % containers.Map: canonical name → sym handle
        EdgeGraph_                         % ir.EdgeGraph handle
        ExecConfig_                        % ir.ExecutionConfig
        ResidualFunc_                      % cached numeric function handle
    end

    % ---- public methods ----
    methods

        %% Constructor: propagate poses and build symbolic residual.
        %   obj = ClosureSolver(EDGEGRAPH, EXECCONFIG, JOINTVARMAP)
        function obj = ClosureSolver(edgeGraph, execConfig, jointVarMap)
            arguments
                edgeGraph    (1,1) ir.EdgeGraph
                execConfig   (1,1) ir.ExecutionConfig
                jointVarMap  containers.Map
            end

            obj.EdgeGraph_ = edgeGraph;
            obj.ExecConfig_ = execConfig;
            obj.JointVarMap_ = jointVarMap;
            obj.Tolerances = execConfig.Tolerances;
            obj.ResidualFunc_ = [];

            % ---- validate closure mode ----
            rawCuts = execConfig.ClosureCuts;
            assert(~isempty(rawCuts), ...
                'solver:ClosureSolver:noCuts', ...
                'ExecutionConfig must contain at least one ClosureCut.');
            assert(strcmp(execConfig.Mode, 'closed_loop'), ...
                'solver:ClosureSolver:wrongMode', ...
                'ExecutionConfig mode must be "closed_loop", got "%s".', ...
                execConfig.Mode);

            % ---- propagate symbolic poses through the edge graph ----
            obj.Poses_ = edgeGraph.propagate();

            % ---- enrich ClosureCuts with T_mate from EdgeGraph ----
            obj.ClosureCuts = obj.enrichClosureCuts(rawCuts);
            obj.NumCuts = numel(obj.ClosureCuts);

            % ---- collect unknown joint variable sym handles ----
            ujv = execConfig.getUnknownJointVars();
            nUnknown = numel(ujv);
            obj.UnknownSyms = sym(zeros(1, nUnknown));
            obj.UnknownNames = cell(1, nUnknown);
            for i = 1:nUnknown
                obj.UnknownSyms(i) = ujv(i).symHandle;
                obj.UnknownNames{i} = ujv(i).name;
            end

            % ---- build and cache full symbolic residual ----
            obj.ResidualSym = obj.buildFullResidual();
            obj.NumComponents = numel(obj.ResidualSym);
        end

        %% buildResidualForCut  Symbolic 6-DOF residual for one closure cut.
        %   res = obj.buildResidualForCut(CUTINDEX)
        %     CUTINDEX : index into obj.ClosureCuts
        %     res      : N×1 sym — residual components for this cut
        function res = buildResidualForCut(obj, cutIndex)
            cut = obj.ClosureCuts(cutIndex);

            % validate frame existence
            assert(isKey(obj.Poses_, cut.near), ...
                'solver:ClosureSolver:frameNotFound', ...
                'Near frame "%s" not in propagated poses.', cut.near);
            assert(isKey(obj.Poses_, cut.far), ...
                'solver:ClosureSolver:frameNotFound', ...
                'Far frame "%s" not in propagated poses.', cut.far);

            T_near = obj.Poses_(cut.near);  % 4×4 sym — socket frame FK pose
            T_far  = obj.Poses_(cut.far);   % 4×4 sym — plug frame FK pose
            M      = cut.T_mate;            % 4×4 double — mate transform (near→far)

            % At closure: T_far = T_near * M
            % Residual in far frame: (T_near * M) \ T_far → I at closure
            T_expected = T_near * M;
            T_err = T_expected \ T_far;  % 4×4 sym

            % decompose to 6-DOF pose error
            resFull = localSymbolicPoseErrorFromRelative(T_err);  % 6×1 sym

            % filter by components mask
            if isfield(cut, 'components') && ~isempty(cut.components)
                comps = cut.components;
                allComps = {'tx','ty','tz','rx','ry','rz'};
                mask = ismember(allComps, comps);
                res = resFull(mask);
            else
                res = resFull;
            end
        end

        %% buildFullResidual  Concatenate residuals from all closure cuts.
        %   res = obj.buildFullResidual()
        function res = buildFullResidual(obj)
            nCuts = numel(obj.ClosureCuts);
            resPieces = cell(nCuts, 1);
            for i = 1:nCuts
                resPieces{i} = obj.buildResidualForCut(i);
            end
            res = vertcat(resPieces{:});
        end

        %% toNumericFunction  Convert symbolic residual to fmincon-ready handle.
        %   fh = obj.toNumericFunction()
        %     fh : @(x) function handle — x is a column vector of unknown
        %          joint values in the order of obj.UnknownSyms.
        function fh = toNumericFunction(obj)
            if ~isempty(obj.ResidualFunc_)
                fh = obj.ResidualFunc_;
                return;
            end
            fh = matlabFunction(obj.ResidualSym, 'Vars', {obj.UnknownSyms.'});
            obj.ResidualFunc_ = fh;
        end

        %% verifyZeroPose  Assert residual ≈ 0 at zero joint configuration.
        function verifyZeroPose(obj)
            zeroVals = zeros(numel(obj.UnknownSyms), 1);
            r = obj.evalResidual(zeroVals);
            rNorm = norm(r);
            assert(rNorm < 1e-12, ...
                'solver:ClosureSolver:zeroPoseFailed', ...
                ['Zero-pose residual norm = %.3e (expected < 1e-12).\n' ...
                 'Residual components: %s'], ...
                rNorm, mat2str(r, 4));
            fprintf('  Zero-pose verification: |r| = %.2e  OK\n', rNorm);
        end

        %% evalResidual  Evaluate numeric residual at given joint values.
        %   r = obj.evalResidual(JOINTVALS)
        %     JOINTVALS : numeric vector (order of obj.UnknownSyms) or
        %                 containers.Map (canonical name → numeric value)
        function r = evalResidual(obj, jointVals)
            if isa(jointVals, 'containers.Map')
                vals = zeros(numel(obj.UnknownSyms), 1);
                for i = 1:numel(obj.UnknownNames)
                    if isKey(jointVals, obj.UnknownNames{i})
                        vals(i) = jointVals(obj.UnknownNames{i});
                    end
                end
            else
                vals = jointVals;
            end
            assert(numel(vals) == numel(obj.UnknownSyms), ...
                'solver:ClosureSolver:valCount', ...
                'Expected %d joint values, got %d.', ...
                numel(obj.UnknownSyms), numel(vals));
            fh = obj.toNumericFunction();
            r = fh(vals(:));
        end

        %% solveIK  Solve IK via fmincon minimizing squared residual norm.
        %   [X_OPT, FVAL, EXITFLAG] = obj.solveIK()
        %   [X_OPT, FVAL, EXITFLAG] = obj.solveIK(X0, OPTIONS)
        function [x_opt, fval, exitflag] = solveIK(obj, x0, options)
            arguments
                obj
                x0      (:,1) double = zeros(numel(obj.UnknownSyms), 1)
                options (1,1) struct = struct()
            end

            if isempty(fieldnames(options))
                options = optimoptions('fmincon', ...
                    'Algorithm', 'sqp', ...
                    'Display', 'iter-detailed', ...
                    'OptimalityTolerance', 1e-12, ...
                    'StepTolerance', 1e-12, ...
                    'MaxFunctionEvaluations', 5000, ...
                    'MaxIterations', 1000);
            end

            fh = obj.toNumericFunction();

            % objective: sum of squared residuals
            objective = @(x) sum(fh(x(:)).^2);

            [x_opt, fval, exitflag] = fmincon(objective, x0, ...
                [], [], [], [], [], [], [], options);
        end

        %% solveIKWithTarget  Solve IK with combined closure + end-frame pose target.
        %   [X_OPT, FVAL, EXITFLAG, REPORT] = obj.solveIKWithTarget(TARGETPOSE, ENDFRAME)
        %   [X_OPT, ...] = obj.solveIKWithTarget(TARGETPOSE, ENDFRAME, X0, OPTIONS)
        %
        %   Builds a combined residual = [closure_residual; pose_error] and
        %   minimises ‖r‖² via fmincon SQP.  The pose error is the 6-DOF
        %   difference between TARGETPOSE and the FK pose of ENDFRAME,
        %   decomposed as [tx,ty,tz,rx,ry,rz] (Z-Y-X Euler).
        %
        %   Inputs:
        %     TARGETPOSE : 4×4 double — target pose for ENDFRAME in world frame
        %     ENDFRAME   : char — fully-qualified frame name (e.g.
        %                  'frame_link_D2.frame_hyper_cube').  Must exist in
        %                  the propagated Poses map.
        %     X0         : initial guess (default: zeros)
        %     OPTIONS    : fmincon options struct (default: SQP, tight tolerances)
        %
        %   Outputs:
        %     X_OPT      : optimal joint values
        %     FVAL       : final objective value ‖r‖²
        %     EXITFLAG   : fmincon exit flag
        %     REPORT     : struct with fields:
        %       .closureResidual  — closure-only residual at solution
        %       .poseResidual     — pose-only residual at solution
        %       .combinedResidual — full combined residual at solution
        %       .closureNorm      — ‖closure residual‖
        %       .poseNorm         — ‖pose residual‖
        function [x_opt, fval, exitflag, report] = solveIKWithTarget(obj, targetPose, endFrame, x0, options)
            arguments
                obj
                targetPose (4,4) double
                endFrame   (1,:) char
                x0         (:,1) double = zeros(numel(obj.UnknownSyms), 1)
                options    (1,1) struct = struct()
            end

            % ---- validate endFrame ----
            assert(isKey(obj.Poses_, endFrame), ...
                'solver:ClosureSolver:endFrameNotFound', ...
                'End frame "%s" not found in propagated poses.', endFrame);

            % ---- build symbolic end-frame pose error ----
            T_end_sym = obj.Poses_(endFrame);       % 4×4 sym
            T_rel = targetPose \ T_end_sym;          % 4×4 sym (mixed: double \ sym)
            poseErrSym = localSymbolicPoseErrorFromRelative(T_rel);  % 6×1 sym

            % ---- build combined residual: closure + pose ----
            combinedSym = [obj.ResidualSym; poseErrSym];

            % ---- convert to numeric function handle ----
            fhCombined = matlabFunction(combinedSym, 'Vars', {obj.UnknownSyms.'});

            % ---- set up fmincon options ----
            if isempty(fieldnames(options))
                options = optimoptions('fmincon', ...
                    'Algorithm', 'sqp', ...
                    'Display', 'iter-detailed', ...
                    'OptimalityTolerance', 1e-12, ...
                    'StepTolerance', 1e-12, ...
                    'MaxFunctionEvaluations', 5000, ...
                    'MaxIterations', 1000);
            end

            % ---- solve ----
            objective = @(x) sum(fhCombined(x(:)).^2);
            [x_opt, fval, exitflag] = fmincon(objective, x0, ...
                [], [], [], [], [], [], [], options);

            % ---- build diagnostic report ----
            if nargout >= 4
                rFull = fhCombined(x_opt(:));
                nClosure = numel(obj.ResidualSym);
                rClosure = rFull(1:nClosure);
                rPose    = rFull(nClosure+1:end);

                report = struct();
                report.closureResidual = rClosure;
                report.poseResidual    = rPose;
                report.combinedResidual = rFull;
                report.closureNorm     = norm(rClosure);
                report.poseNorm        = norm(rPose);
            end
        end

        %% printResidualReport  Print a human-readable residual breakdown.
        function printResidualReport(obj, jointVals)
            r = obj.evalResidual(jointVals);
            rNorm = norm(r);

            if isa(jointVals, 'containers.Map')
                fprintf('\n=== Closure Residual Report ===\n');
                fprintf('Joint values:\n');
                for i = 1:numel(obj.UnknownNames)
                    if isKey(jointVals, obj.UnknownNames{i})
                        fprintf('  %-25s = % 12.6f\n', ...
                            obj.UnknownNames{i}, jointVals(obj.UnknownNames{i}));
                    end
                end
            else
                fprintf('\n=== Closure Residual Report ===\n');
                fprintf('Joint values (order: %s):\n', strjoin(obj.UnknownNames, ', '));
                fprintf('  [%s]\n', strjoin(arrayfun(@(v) sprintf('% 12.6f', v), ...
                    jointVals(:), 'UniformOutput', false), ', '));
            end

            fprintf('\nResidual components:\n');
            compIdx = 1;
            allComps = {'tx','ty','tz','rx','ry','rz'};
            for i = 1:obj.NumCuts
                cut = obj.ClosureCuts(i);
                comps = cut.components;
                fprintf('  Cut %d (%s ←→ %s):\n', i, cut.near, cut.far);
                for j = 1:numel(comps)
                    fprintf('    %-4s = % 12.6e\n', comps{j}, r(compIdx));
                    compIdx = compIdx + 1;
                end
            end
            fprintf('  |r| = %.3e\n', rNorm);
            fprintf('  Translation tolerance: %.1e mm\n', obj.Tolerances.translation_mm);
            fprintf('  Rotation tolerance:    %.1e rad\n', obj.Tolerances.rotation_rad);

            % Collect component labels for this run
            compLabels = {};
            for i = 1:obj.NumCuts
                compLabels = [compLabels, obj.ClosureCuts(i).components]; %#ok<AGROW>
            end
            tMask = contains(compLabels, 't');
            rMask = contains(compLabels, 'r');
            if any(tMask), tErr = norm(r(tMask)); else tErr = 0; end
            if any(rMask), rErr = norm(r(rMask)); else rErr = 0; end
            fprintf('  |t_err| = %.3e mm, |r_err| = %.3e rad\n', tErr, rErr);
        end

    end

    % ---- private methods ----
    methods (Access = private)

        %% enrichClosureCuts  Augment ClosureCuts with T_mate from EdgeGraph.
        function cuts = enrichClosureCuts(obj, rawCuts)
            cuts = rawCuts;
            allMates = obj.EdgeGraph_.findMates();
            closedMask = strcmp({allMates.kind}, 'closed_mate');
            closedMates = allMates(closedMask);

            for i = 1:numel(cuts)
                % match closed_mate edge by near/far frame names
                found = false;
                for j = 1:numel(closedMates)
                    if strcmp(closedMates(j).from, cuts(i).near) && ...
                       strcmp(closedMates(j).to, cuts(i).far)
                        cuts(i).T_mate = closedMates(j).T;
                        found = true;
                        break;
                    end
                end
                assert(found, ...
                    'solver:ClosureSolver:mateNotFound', ...
                    ['No closed_mate edge matches cut %d (near="%s", far="%s"). ' ...
                     'Ensure EdgeGraph has not been modified after ExecutionConfig construction.'], ...
                    i, cuts(i).near, cuts(i).far);
            end
        end

    end

end

%% ---- local functions (not methods) ----

function err = localSymbolicPoseErrorFromRelative(T_rel)
%LOCALSYMBOLICPOSEERRORFROMRELATIVE  6-DOF pose error from relative transform.
%   T_rel is a 4×4 homogeneous transform (sym or double) that should be
%   identity when the poses match.  Decomposes to [tx,ty,tz,rx,ry,rz] where
%   translation is in length units (mm) and rotation is Z-Y-X Euler angles (rad).
%
%   Uses atan2 from Symbolic Math Toolbox, which accepts sym inputs.

    % position error
    p_err = T_rel(1:3, 4);

    % rotation error: extract Z-Y-X Euler angles from R_rel
    R_rel = T_rel(1:3, 1:3);

    % Z-Y-X decomposition: R = Rz(rz) * Ry(ry) * Rx(rx)
    %   R(3,1) = -sin(ry)  →  ry = atan2(-R(3,1), sqrt(R(1,1)^2+R(2,1)^2))
    %   R(3,2) =  cos(ry)*sin(rx)  →  rx = atan2(R(3,2), R(3,3))
    %   R(2,1) =  sin(rz)*cos(ry)  →  rz = atan2(R(2,1), R(1,1))
    ry = atan2(-R_rel(3,1), sqrt(R_rel(1,1)^2 + R_rel(2,1)^2));
    rx = atan2(R_rel(3,2), R_rel(3,3));
    rz = atan2(R_rel(2,1), R_rel(1,1));

    err = [p_err; rx; ry; rz];
end
