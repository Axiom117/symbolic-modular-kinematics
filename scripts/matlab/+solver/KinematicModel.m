classdef KinematicModel < handle
%KINEMATICMODEL  Symbolic kinematic model: FK expression + eval + problem formulation.
%   Queries a symbolically-expanded EdgeGraph (built by ir.Expander in the
%   pure symbolic pipeline, A.4.0) for a named end frame, decomposes the
%   4×4 homogeneous pose into position and rotation symbolic expressions,
%   provides eval() for fast numeric evaluation, and formulateProblem() to
%   construct FK/IK solving problems from L3 execution config.
%
%   KINEMATICMODEL is not a solver — it stores the symbolic forward kinematics
%   mapping q → T_end(q), evaluates it numerically, and wires it to the
%   ExecutionConfig variable partition to produce solver-ready function handles.
%
%   Usage:
%       e = ir.Expander(dslYaml);                   % pure symbolic expansion
%       km = solver.KinematicModel(e.EdgeGraph_, 'pipette.tip_origin');
%       % km.TSym     — 4×4 sym  homogeneous transform (world→endFrame)
%       % km.PosExpr  — 3×1 sym  position [x; y; z]
%       % km.RotExpr  — 3×3 sym  rotation matrix
%       % km.JointVars — sym array of all joint variables on the path
%
%       % Evaluate at specific joint values:
%       T_num = double(subs(km.TSym, km.JointVars, [0.5236; -0.7854]));
%
%       % Formulate solving problem with execution config:
%       cfg = ir.ExecutionConfig('exec-config.yaml', e.SymbolRegistry);
%       prob = km.formulateProblem(cfg);
%       % prob.Type       — 'FK' or 'IK'
%       % prob.eval(vals) — for FK: returns 4×4 pose; for IK: returns residual vector
%
%   See also: +ir/EdgeGraph, +ir/Expander, +core/PosePropagator, +ir/ExecutionConfig

    % ---- public read-only properties ----
    properties (SetAccess = private)
        TSym          % 4×4 sym — world→endFrame homogeneous transform
        PosExpr       % 3×1 sym — position component of TSym
        RotExpr       % 3×3 sym — rotation component of TSym
        JointVars     % sym array — all symbolic joint variables appearing in TSym
        EndFrame      (1,:) char  % name of the target frame
    end

    % ---- private properties ----
    properties (Access = private)
        JointVarMap_  % containers.Map: canonical name ('joint1.q') → sym handle
    end

    % ---- public methods ----
    methods

        %% Constructor: propagate FK and extract the end-frame symbolic pose.
        %   obj = KinematicModel(EDGEGRAPH, ENDFRAME)
        %     EDGEGRAPH : ir.EdgeGraph with symbolic joint edges (built by
        %                 ir.Expander in the pure symbolic pipeline).
        %                 Fixed and mate edges may be numeric (double) —
        %                 they are auto-promoted during propagation.
        %     ENDFRAME  : char — fully-qualified frame name (e.g.
        %                 'pipette.tip_origin') reachable from ground nodes
        %                 via spanning-tree edges.
        function obj = KinematicModel(edgeGraph, endFrame, jointVarMap)
            arguments
                edgeGraph    (1,1) ir.EdgeGraph
                endFrame     (1,:) char
                jointVarMap  containers.Map = containers.Map('KeyType','char','ValueType','any')
            end

            obj.EndFrame = endFrame;
            obj.JointVarMap_ = jointVarMap;

            % propagate poses through the symbolic edge graph.
            % PosePropagator.propagatePoses handles mixed double/sym T matrices
            % transparently; closed_mate edges are excluded by toStruct().
            poses = edgeGraph.propagate();

            assert(isKey(poses, endFrame), ...
                'solver:KinematicModel:endFrameNotFound', ...
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
        %            obj.JointVars (symvar alphabetical order).
        %   T = EVAL(obj, VMAP)
        %     VMAP : containers.Map — canonical name → numeric value
        %            (e.g. keys('joint1.q') = 0.5236).
        %            Requires JointVarMap passed to constructor.
        %     T    : 4×4 double — numeric homogeneous transform.
        function T = eval(obj, vals)
            arguments
                obj
                vals
            end
            valsNum = obj.resolveJointValues(vals);
            assert(numel(valsNum) == numel(obj.JointVars), ...
                'solver:KinematicModel:valCount', ...
                'Expected %d joint values, got %d.', ...
                numel(obj.JointVars), numel(valsNum));
            T = double(subs(obj.TSym, obj.JointVars, reshape(valsNum, size(obj.JointVars))));
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

        %% formulateProblem  Construct solving problem from execution config.
        %   prob = obj.formulateProblem(EXECCONFIG)
        %     EXECCONFIG : ir.ExecutionConfig — validated L3 execution config
        %     prob       : struct with fields:
        %       .Type       — 'FK' (evaluate pose) or 'IK' (solve residual)
        %       .eval(vals) — function handle:
        %           FK: vals = joint values → returns 4×4 double pose
        %           IK: vals = joint values → returns residual vector (6×1)
        %       .JointVarNames — cell array of joint variable canonical names
        %       .JointVarOrder — sym array: joint vars in the order expected by eval()
        %       .TargetPose    — 4×4 double (IK only): the desired end-effector pose
        %
        %   For open_loop FK mode:
        %     prob = tf.formulateProblem(cfg);
        %     T_tip = prob.eval([q1_val; q2_val]);  % numeric pose
        %
        %   For closed_loop IK mode (requires target pose set in cfg or passed separately):
        %     prob = tf.formulateProblem(cfg);
        %     residual = prob.eval([q1; q2; ...]);  % 6×1 pose error
        function prob = formulateProblem(obj, execConfig, targetPose)
            arguments
                obj
                execConfig   (1,1) ir.ExecutionConfig
                targetPose   (4,4) double = eye(4)  % IK target; default identity
            end

            dir = execConfig.getSolvingDirection();

            % -- collect joint variable ordering from config --
            switch dir
                case 'FK'
                    % known: joint vars → unknown: endFrame pose
                    kjv = execConfig.getKnownJointVars();
                case 'IK'
                    % known: target pose → unknown: joint vars
                    kjv = execConfig.getUnknownJointVars();
            end

            % build ordered joint var list
            nJoints = numel(kjv);
            jointNames = cell(1, nJoints);
            jointSyms = sym(zeros(1, nJoints));
            for i = 1:nJoints
                jointNames{i} = kjv(i).name;
                jointSyms(i) = kjv(i).symHandle;
            end

            % -- verify all required joint vars appear in TSym --
            tsVars = symvar(obj.TSym);
            for i = 1:nJoints
                assert(ismember(jointSyms(i), tsVars), ...
                    'solver:KinematicModel:jointNotInPath', ...
                    'Joint variable "%s" does not appear in TSym — check FK path.', ...
                    jointNames{i});
            end

            % -- validate joint var count matches expected order --
            assert(numel(jointSyms) == numel(obj.JointVars) || strcmp(dir, 'FK'), ...
                'solver:KinematicModel:jointVarMismatch', ...
                ['Expected %d joint vars in problem formulation, but ' ...
                 'obj.JointVars has %d. Check known/unknown partition.'], ...
                numel(jointSyms), numel(obj.JointVars));

            prob = struct();
            prob.Type = dir;
            prob.JointVarNames = jointNames;
            prob.JointVarOrder = jointSyms;

            switch dir
                case 'FK'
                    % FK: substitute joint values → numeric pose
                    prob.eval = @(vals) localEvalFK(obj, jointSyms, vals);
                    prob.TargetPose = [];

                case 'IK'
                    % IK: compute 6-DOF pose error residual
                    prob.TargetPose = targetPose;
                    prob.eval = @(vals) localEvalIKResidual(obj, jointSyms, vals, targetPose);
            end
        end

    end

    % ---- private helpers ----
    methods (Access = private)

        %% resolveJointValues  Convert named or positional vals to numeric vector.
        %   valsNum = resolveJointValues(obj, vals)
        %     If vals is a containers.Map, resolves canonical names → indices
        %     via the stored JointVarMap_.  Otherwise passes through as-is
        %     (positional numeric vector).
        function valsNum = resolveJointValues(obj, vals)
            if isa(vals, 'containers.Map')
                assert(obj.JointVarMap_.Count > 0, ...
                    'solver:KinematicModel:noJointVarMap', ...
                    ['Named eval requires JointVarMap. ' ...
                     'Pass Expander.JointVarMap to KinematicModel constructor.']);
                valsNum = zeros(size(obj.JointVars));
                jvKeys = keys(obj.JointVarMap_);
                for i = 1:numel(jvKeys)
                    if isKey(vals, jvKeys{i})
                        symHandle = obj.JointVarMap_(jvKeys{i});
                        idx = find(obj.JointVars == symHandle, 1);
                        if ~isempty(idx)
                            valsNum(idx) = vals(jvKeys{i});
                        end
                    end
                end
            else
                valsNum = vals;
            end
        end

    end

end

%% ---- local functions (not methods) ----

function T = localEvalFK(km, jointSyms, vals)
    assert(numel(vals) == numel(jointSyms), ...
        'solver:KinematicModel:FKValCount', ...
        'Expected %d joint values for FK, got %d.', ...
        numel(jointSyms), numel(vals));
    T = double(subs(km.TSym, jointSyms, reshape(vals, size(jointSyms))));
end

function residual = localEvalIKResidual(km, jointSyms, vals, T_des)
    assert(numel(vals) == numel(jointSyms), ...
        'solver:KinematicModel:IKValCount', ...
        'Expected %d joint values for IK residual, got %d.', ...
        numel(jointSyms), numel(vals));
    T_cur = double(subs(km.TSym, jointSyms, reshape(vals, size(jointSyms))));
    residual = localPoseError(T_cur, T_des);
end

function err = localPoseError(T_cur, T_des)
%LOCALPOSEERROR  6-DOF pose error: translation (mm) + Z-Y-X Euler angle (rad).
%   err = [tx; ty; tz; rx; ry; rz]
%   where tx,ty,tz = position error (mm), rx,ry,rz = Z-Y-X Euler angle error (rad).

    % position error: T_des^{-1} * T_cur → translation component
    T_err = T_des \ T_cur;  % = inv(T_des) * T_cur
    p_err = T_err(1:3, 4);

    % rotation error: extract Z-Y-X Euler angles from R_err = R_des' * R_cur
    R_cur = T_cur(1:3, 1:3);
    R_des = T_des(1:3, 1:3);
    R_err = R_des' * R_cur;

    % Z-Y-X Euler angle extraction: R = Rz(rz) * Ry(ry) * Rx(rx)
    % From R_err:
    ry = atan2(-R_err(3,1), sqrt(R_err(1,1)^2 + R_err(2,1)^2));
    if abs(cos(ry)) > 1e-10
        rx = atan2(R_err(3,2)/cos(ry), R_err(3,3)/cos(ry));
        rz = atan2(R_err(2,1)/cos(ry), R_err(1,1)/cos(ry));
    else
        % gimbal lock: ry ≈ ±pi/2
        rx = 0;
        rz = atan2(R_err(1,2), R_err(2,2));
    end

    err = [p_err; rx; ry; rz];
end
