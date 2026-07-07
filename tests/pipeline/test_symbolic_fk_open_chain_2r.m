%% test_symbolic_fk_open_chain_2r.m
% End-to-end test: DSL → symbolic IR → symbolic FK → numeric verification.
%
% Verifies the pure symbolic pipeline (A.4.0): ir.Expander always produces
% symbolic Poses; evaluateNumeric() substitutes joint values from config.
% The symbolic FK output, when evaluated at joint_config.yaml values,
% matches numeric FK to within 1e-12.
%
% Requires: Symbolic Math Toolbox
%
% Usage:
%   addpath(genpath('../../scripts/matlab'));
%   run('test_symbolic_fk_open_chain_2r.m');

fprintf('=== Symbolic FK Pipeline Test: open-chain-2r ===\n\n');

%% Paths
dslFile    = '../../specs/dsl/examples/open-chain-2r/robot_description.yaml';
configFile = '../../specs/dsl/examples/open-chain-2r/joint_config.yaml';
modLib     = '../../specs/modules/';

%% 1. Build numeric reference (symbolic expansion + evaluateNumeric)
fprintf('1. Building numeric reference pipeline ... ');
eNum = ir.Expander(dslFile);
numPoses = eNum.evaluateNumeric(configFile);
fprintf('OK (%d instances, %d edges, %d poses)\n', ...
    numel(eNum.Instances), eNum.EdgeGraph_.numEdges, numPoses.Count);

endFrame = 'pipette.tip_origin';
assert(isKey(numPoses, endFrame), 'End frame not in numeric poses.');
T_num_ref = numPoses(endFrame);
fprintf('   Numeric T_end (ref):\n');
disp(T_num_ref);

%% 2. Build symbolic pipeline (pure symbolic, always-on)
fprintf('\n2. Building symbolic pipeline ... ');
try
    eSym = ir.Expander(dslFile);
catch ME
    if contains(ME.message, 'Unrecognized function or variable') && contains(ME.message, 'sym')
        error('Symbolic Math Toolbox required. Install it to run this test.');
    end
    rethrow(ME);   % let ALL other errors through unfiltered
end
fprintf('OK\n');

%% 3. Verify JointVarMap contains expected joint variables
fprintf('\n3. Checking JointVarMap ... ');
jvKeys = keys(eSym.JointVarMap);
fprintf('%d joint variables in JointVarMap: %s\n', ...
    numel(jvKeys), strjoin(jvKeys, ', '));
assert(numel(jvKeys) >= 2, 'Expected at least 2 joint variables in JointVarMap.');
assert(isKey(eSym.JointVarMap, 'joint1.q'), 'joint1.q missing from JointVarMap.');
assert(isKey(eSym.JointVarMap, 'joint2.q'), 'joint2.q missing from JointVarMap.');

%% 4. Run TaskFrame
fprintf('\n4. Running TaskFrame to endFrame="%s" ... ', endFrame);
tf = ir.TaskFrame(eSym.EdgeGraph_, endFrame);
fprintf('OK\n');
fprintf('   JointVars: %s\n', strjoin(string(tf.JointVars), ', '));
fprintf('   TSym type: %s, size: %dx%d\n', class(tf.TSym), size(tf.TSym));

%% 5. Verify TSym is symbolic (contains trig terms)
fprintf('\n5. Verifying TSym is symbolic ... ');
assert(isa(tf.TSym, 'sym'), 'TSym must be sym type.');
ts = char(tf.TSym);
assert(contains(ts, 'cos') || contains(ts, 'sin'), ...
    'TSym should contain trig terms from revolute joints.');
fprintf('OK (contains trig functions)\n');

%% 6. Evaluate symbolic FK at joint_config.yaml values and compare to numeric ref
fprintf('\n6. Evaluating symbolic FK at joint_config values ...\n');

% Read joint values from config
jointCfg = core.readYaml(configFile);
q1_val = jointCfg.joint1.q;   % 0.5236
q2_val = jointCfg.joint2.q;   % -0.7854

fprintf('   joint1.q = %.6f rad, joint2.q = %.6f rad\n', q1_val, q2_val);

% symvar returns vars in alphabetical order: joint1_q, joint2_q
% (MATLAB makeValidName replaces '.' with '_' in sym variable names)
vals_numeric = [q1_val; q2_val];
T_num_from_sym = tf.eval(vals_numeric);

fprintf('   T_end from symbolic FK (evaluated):\n');
disp(T_num_from_sym);

%% 7. Compare: error must be < 1e-12
fprintf('\n7. Comparing numeric vs symbolic FK ... ');
diffMat = T_num_ref - T_num_from_sym;
maxErr = max(abs(diffMat(:)));
fprintf('max |error| = %.2e\n', maxErr);
assert(maxErr < 1e-12, ...
    'Symbolic FK evaluation deviates from numeric FK by %.2e (threshold 1e-12).', maxErr);

%% 8. Verify position and rotation decomposition
fprintf('\n8. Checking PosExpr / RotExpr decomposition ... ');
p_sym = tf.evalPos(vals_numeric);
R_sym = tf.evalRot(vals_numeric);
assert(norm(p_sym - T_num_ref(1:3,4)) < 1e-12, 'PosExpr mismatch.');
assert(norm(R_sym - T_num_ref(1:3,1:3)) < 1e-12, 'RotExpr mismatch.');
fprintf('OK (position + rotation match)\n');

%% Done
fprintf('\n=== ALL CHECKS PASSED ===\n');
