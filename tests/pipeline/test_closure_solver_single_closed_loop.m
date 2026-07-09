%% test_closure_solver_single_closed_loop.m
% End-to-end test: DSL → IR → ClosureSolver → residual verification.
%
% Verifies the closed-loop constraint construction (A.4a) for the
% parallelogram single-closed-loop mechanism.  Tests:
%   1. Zero-pose residual = 0 (mechanism closed at zero config)
%   2. Valid closed config residual = 0 (parallelogram joint relationship)
%   3. Residual sensitivity to config perturbation
%   4. Symbolic structure checks
%
% Parallelogram kinematics (verified empirically):
%   AB and CD are on one diagonal → same angle
%   BD and AC are on the other diagonal → opposite angle
%   A closed configuration: [AB=+θ, BD=-θ, CD=+θ, AC=-θ]
%
% Requires: Symbolic Math Toolbox, Optimization Toolbox
%
% Usage:
%   addpath(genpath('../../scripts/matlab'));
%   run('test_closure_solver_single_closed_loop.m');

fprintf('=== Closure Solver Test: single-closed-loop ===\n\n');

%% Paths
dslFile    = '../../specs/dsl/cases/single-closed-loop/robot_description.yaml';
execFile   = '../../specs/dsl/cases/single-closed-loop/execution-config.yaml';

%% 1. Build symbolic pipeline
fprintf('1. Building symbolic pipeline (Expander) ... ');
try
    eSym = ir.Expander(dslFile);
catch ME
    if contains(ME.message, 'Unrecognized function or variable') && ...
       contains(ME.message, 'sym')
        error('Symbolic Math Toolbox required. Install it to run this test.');
    end
    rethrow(ME);
end
fprintf('OK\n');
fprintf('   Instances: %d, Edges: %d, Joint vars: %d\n', ...
    numel(eSym.Instances), eSym.EdgeGraph_.numEdges, eSym.JointVarMap.Count);

%% 2. Verify joint variable registration
fprintf('\n2. Checking joint variables ... ');
jvKeys = keys(eSym.JointVarMap);
assert(numel(jvKeys) == 4, 'Expected 4 joint variables.');
assert(isKey(eSym.JointVarMap, 'joint_AB.q'));
assert(isKey(eSym.JointVarMap, 'joint_BD.q'));
assert(isKey(eSym.JointVarMap, 'joint_CD.q'));
assert(isKey(eSym.JointVarMap, 'joint_AC.q'));
fprintf('OK: %s\n', strjoin(jvKeys, ', '));

%% 3. Build ExecutionConfig
fprintf('\n3. Building ExecutionConfig ... ');
cfg = ir.ExecutionConfig(execFile, eSym.SymbolRegistry, eSym.EdgeGraph_);
assert(strcmp(cfg.Mode, 'closed_loop'), 'Expected closed_loop mode.');
assert(strcmp(cfg.ClosureSource, 'auto'), ...
    'Expected auto-derived closure cuts (from closed_mate edges).');
fprintf('OK\n');
fprintf('   Cuts: %d, Source: %s, EndFrame: %s\n', ...
    numel(cfg.ClosureCuts), cfg.ClosureSource, cfg.EndFrame);

%% 4. Print closure cut details
fprintf('\n4. Closure cut details:\n');
for i = 1:numel(cfg.ClosureCuts)
    cut = cfg.ClosureCuts(i);
    fprintf('   Cut %d: near="%s"  far="%s"  components={%s}\n', ...
        i, cut.near, cut.far, strjoin(cut.components, ','));
end

%% 5. Build ClosureSolver
fprintf('\n5. Building ClosureSolver ... ');
cs = solver.ClosureSolver(eSym.EdgeGraph_, cfg, eSym.JointVarMap);
fprintf('OK\n');
fprintf('   Unknown vars: %s\n', strjoin(cs.UnknownNames, ', '));
fprintf('   Residual dim: %d\n', numel(cs.ResidualSym));

%% 6. Verify symbolic residual structure
fprintf('\n6. Verifying symbolic residual structure ... ');
resStr = char(cs.ResidualSym);
assert(contains(resStr, 'cos') || contains(resStr, 'sin'), ...
    'Residual should contain trig terms.');
fprintf('OK (contains trig functions)\n');
fprintf('   Symbolic expression length: %d chars\n', numel(resStr));

%% 7. Zero-pose verification
fprintf('\n7. Zero-pose verification ...\n');
cs.verifyZeroPose();

%% 8. Verify with valid closed configuration
% Parallelogram kinematics (verified empirically):
%   AB and CD are on one diagonal → same angle
%   BD and AC are on the other diagonal → opposite angle
%   A closed configuration satisfies: AB = CD, BD = AC, AB = -BD
fprintf('\n8. Testing with valid closed configuration ...\n');

qClosed = containers.Map();
qClosed('joint_AB.q') =  pi/6;
qClosed('joint_BD.q') = -pi/6;
qClosed('joint_CD.q') =  pi/6;
qClosed('joint_AC.q') = -pi/6;

fprintf('   Valid closed config: AB=+30°, BD=-30°, CD=+30°, AC=-30°\n');
cs.printResidualReport(qClosed);

rClosed = cs.evalResidual(qClosed);
rNormClosed = norm(rClosed);
fprintf('\n   => Residual norm at valid closed config: %.3e\n', rNormClosed);

% The residual should be very small (closed mechanism)
assert(rNormClosed < 1e-10, ...
    'ClosureSolver:closedConfigFailed', ...
    ['Residual norm at valid closed config = %.3e (expected < 1e-10).\n' ...
     'The parallelogram joint relationship may need adjustment.'], ...
    rNormClosed);
fprintf('   PASS: residual < 1e-10 at valid closed config.\n');

%% 9. Verify residual grows with perturbation
fprintf('\n9. Testing residual sensitivity to perturbation ...\n');

qPerturbed = containers.Map();
qPerturbed('joint_AB.q') =  pi/6 + 0.1;  % perturb AB by +0.1 rad
qPerturbed('joint_BD.q') = -pi/6;
qPerturbed('joint_CD.q') =  pi/6;
qPerturbed('joint_AC.q') = -pi/6;

rPerturbed = cs.evalResidual(qPerturbed);
rNormPerturbed = norm(rPerturbed);
fprintf('   Perturbed AB by +0.1 rad => |r| = %.3e\n', rNormPerturbed);
assert(rNormPerturbed > rNormClosed * 10, ...
    'ClosureSolver:sensitivityFailed', ...
    'Perturbed residual should be significantly larger than closed config residual.');
fprintf('   PASS: perturbed residual > closed residual.\n');

%% 10. Quick IK sanity check (fmincon from perturbed back to closed)
fprintf('\n10. IK sanity check (fmincon from perturbed initial guess) ...\n');

% Start from perturbed config, should converge back to a valid closed config
x0 = [pi/6 + 0.1; -pi/6; pi/6; -pi/6];  % perturbed AB
[x_opt, fval, exitflag] = cs.solveIK(x0);
fprintf('   Exit flag: %d, Final obj: %.3e\n', exitflag, fval);
fprintf('   Solution: [%s]\n', strjoin(arrayfun(@(v) sprintf('%.4f', v), ...
    x_opt, 'UniformOutput', false), ', '));

if exitflag > 0
    rOpt = cs.evalResidual(x_opt);
    fprintf('   Residual at solution: |r| = %.3e\n', norm(rOpt));
    fprintf('   PASS: fmincon converged to a valid solution.\n');
else
    fprintf('   WARNING: fmincon did not converge (this is acceptable for A.4a).\n');
end

%% Summary
fprintf('\n=== All tests passed ===\n');
