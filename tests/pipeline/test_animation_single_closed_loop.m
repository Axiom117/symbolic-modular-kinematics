%% test_animation_single_closed_loop.m
% Animation test for single-closed-loop parallelogram mechanism.
%
% Generates a smooth joint trajectory that sweeps through valid closed
% configurations of the parallelogram, then animates via viz.animate.
%
% Parallelogram closure: AB=CD, BD=AC, AB=-BD
%   Trajectory: [AB=θ(t), BD=-θ(t), CD=θ(t), AC=-θ(t)]
%   where θ(t) sweeps from 0 → +30° → 0 → -30° → 0
%
% Requires: Symbolic Math Toolbox
%
% Usage:
%   addpath(genpath('../../scripts/matlab'));
%   run('test_animation_single_closed_loop.m');
%   % Press ▶ Play in the figure to start animation

fprintf('=== Animation Test: single-closed-loop ===\n\n');

%% Paths
dslFile = '../../specs/dsl/cases/single-closed-loop/robot_description.yaml';

%% Generate joint trajectory
% Closed-form sweep: one full cycle of the parallelogram
nFrames = 30;
t = linspace(0, 2*pi, nFrames);
theta = (pi/6) * sin(t);  % ±30° sinusoidal sweep

% Build trajectory matrix [AB, BD, CD, AC]
Q = [theta; -theta; theta; -theta]';

varNames = {'joint_AB.q', 'joint_BD.q', 'joint_CD.q', 'joint_AC.q'};

fprintf('Trajectory: %d frames, joints: %s\n', nFrames, strjoin(varNames, ', '));
fprintf('Angle range: [%.1f°, %.1f°]\n', rad2deg(min(theta)), rad2deg(max(theta)));

%% Verify closure at sample frames
fprintf('\nVerifying closure at sample frames ...\n');
eSym = ir.Expander(dslFile);
cfg = ir.ExecutionConfig( ...
    '../../specs/dsl/cases/single-closed-loop/execution-config.yaml', ...
    eSym.SymbolRegistry, eSym.EdgeGraph_);
cs = solver.ClosureSolver(eSym.EdgeGraph_, cfg, eSym.JointVarMap);

sampleFrames = [1, round(nFrames/4), round(nFrames/2), round(3*nFrames/4)];
allOk = true;
for idx = sampleFrames
    qMap = containers.Map();
    for j = 1:numel(varNames)
        qMap(varNames{j}) = Q(idx, j);
    end
    r = cs.evalResidual(qMap);
    rNorm = norm(r);
    status = core.CommonUtils.tern(rNorm < 1e-10, 'OK', 'FAIL');
    if rNorm >= 1e-10; allOk = false; end
    fprintf('  Frame %3d: θ=%.1f°  |r|=%.2e  %s\n', idx, rad2deg(theta(idx)), rNorm, status);
end
assert(allOk, 'Closure residual too large at one or more sample frames.');

%% Run animation
fprintf('\nLaunching animation (press ▶ Play to start, close figure to continue) ...\n');
viz.animate(dslFile, Q, varNames, 15);  % 15 fps

fprintf('Animation complete.\n');
fprintf('\n=== Animation test passed ===\n');
