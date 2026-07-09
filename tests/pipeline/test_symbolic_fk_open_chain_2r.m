%% test_symbolic_fk_open_chain_2r.m
% End-to-end test: DSL → symbolic IR → symbolic FK → visualization.
%
% Verifies the pure symbolic pipeline (A.4.0): ir.Expander always produces
% symbolic Poses; JointVarMap tracks joint variables; TaskFrame extracts
% the end-frame symbolic pose with position/rotation decomposition;
% visualization confirms the mechanism geometry.
%
% Requires: Symbolic Math Toolbox
%
% Usage:
%   addpath(genpath('../../scripts/matlab'));
%   run('test_symbolic_fk_open_chain_2r.m');

fprintf('=== Symbolic FK Pipeline Test: open-chain-2r ===\n\n');

%% Paths
dslFile    = '../../specs/dsl/cases/open-chain-2r/robot_description.yaml';
configFile = '../../specs/dsl/cases/open-chain-2r/joint_config.yaml';

%% 1. Build symbolic pipeline (pure symbolic expansion)
fprintf('1. Building symbolic pipeline ... ');
try
    eSym = ir.Expander(dslFile);
catch ME
    if contains(ME.message, 'Unrecognized function or variable') && contains(ME.message, 'sym')
        error('Symbolic Math Toolbox required. Install it to run this test.');
    end
    rethrow(ME);
end
fprintf('OK (%d instances, %d edges)\n', ...
    numel(eSym.Instances), eSym.EdgeGraph_.numEdges);

%% 2. Verify JointVarMap contains expected joint variables
fprintf('\n2. Checking JointVarMap ... ');
jvKeys = keys(eSym.JointVarMap);
fprintf('%d joint variables: %s\n', ...
    numel(jvKeys), strjoin(jvKeys, ', '));
assert(numel(jvKeys) >= 2, 'Expected at least 2 joint variables.');
assert(isKey(eSym.JointVarMap, 'joint1.q'), 'joint1.q missing.');
assert(isKey(eSym.JointVarMap, 'joint2.q'), 'joint2.q missing.');

%% 3. Run TaskFrame to extract end-frame symbolic pose
endFrame = 'frame2.frame_hyper_cube';
fprintf('\n3. Running TaskFrame to endFrame="%s" ... ', endFrame);
tf = ir.TaskFrame(eSym.EdgeGraph_, endFrame);
fprintf('OK\n');
fprintf('   JointVars: %s\n', strjoin(string(tf.JointVars), ', '));
fprintf('   TSym type: %s, size: %dx%d\n', class(tf.TSym), size(tf.TSym));

%% 4. Verify TSym is symbolic (structural checks)
fprintf('\n4. Verifying TSym structure ... ');
assert(isa(tf.TSym, 'sym'), 'TSym must be sym type.');
ts = char(tf.TSym);
assert(contains(ts, 'cos') || contains(ts, 'sin'), ...
    'TSym should contain trig terms from revolute joints.');
fprintf('OK (contains trig functions)\n');

%% 5. Position / rotation decomposition
fprintf('\n5. Checking PosExpr / RotExpr decomposition ... ');
assert(isequal(size(tf.PosExpr), [3 1]), 'PosExpr must be 3×1.');
assert(isequal(size(tf.RotExpr), [3 3]), 'RotExpr must be 3×3.');
assert(isa(tf.PosExpr, 'sym'), 'PosExpr must be sym.');
assert(isa(tf.RotExpr, 'sym'), 'RotExpr must be sym.');

% Self-consistency: TSym(1:3,4) == PosExpr, TSym(1:3,1:3) == RotExpr
assert(isequal(tf.TSym(1:3,4), tf.PosExpr), 'TSym(1:3,4) ≠ PosExpr.');
assert(isequal(tf.TSym(1:3,1:3), tf.RotExpr), 'TSym(1:3,1:3) ≠ RotExpr.');

% Evaluate at joint_config values as a numeric sanity check
jointCfg = core.readYaml(configFile);
q1_val = jointCfg.joint1.q;
q2_val = jointCfg.joint2.q;
vals_numeric = [q1_val; q2_val];

T_num = tf.eval(vals_numeric);
p_num = tf.evalPos(vals_numeric);
R_num = tf.evalRot(vals_numeric);

fprintf('OK\n');
fprintf('   joint1.q = %.6f rad, joint2.q = %.6f rad\n', q1_val, q2_val);
fprintf('   T_end (evaluated):\n');
disp(T_num);

% Rotation matrix orthonormality check
assert(abs(det(R_num) - 1) < 1e-12, 'Rotation matrix det ≠ 1.');
assert(norm(R_num * R_num' - eye(3), 'fro') < 1e-12, 'R * R'' ≠ I.');
fprintf('   Rotation matrix: orthonormal (det=1, R*R''=I)\n');

%% 6. Visualization
fprintf('\n6. Visualizing mechanism ... ');
viz.mechanism(dslFile, configFile);
fprintf('OK (figure opened)\n');

%% Done
fprintf('\n=== ALL CHECKS PASSED ===\n');
