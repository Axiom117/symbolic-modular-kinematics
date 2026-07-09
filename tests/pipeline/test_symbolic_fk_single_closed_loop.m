%% test_symbolic_fk_single_closed_loop.m
% End-to-end test: DSL → symbolic IR → symbolic FK → visualization.
%
% Verifies the pure symbolic pipeline (A.4.0) for a parallelogram single-closed-loop
% mechanism: Link A (top, 3 frames) → Link B (right, 2 frames) → Link D (bottom, 3 frames) →
% Link C (left, 2 frames) along the spanning tree; 4 revolute joints (AB, BD, CD, AC);
% each Joint uses paired roll (1 / -1) to align the rotation axis to X without
% affecting downstream module orientations. The closed chord (joint_AC → Link A)
% is excluded from FK propagation but registered as a loop-closure constraint.
%
% Requires: Symbolic Math Toolbox
%
% Usage:
%   addpath(genpath('../../scripts/matlab'));
%   run('test_symbolic_fk_single_closed_loop.m');

fprintf('=== Symbolic FK Pipeline Test: single-closed-loop ===\n\n');

%% Paths
dslFile    = '../../specs/dsl/cases/single-closed-loop/robot_description.yaml';
configFile = '../../specs/dsl/cases/single-closed-loop/joint_config.yaml';

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

%% 2. Verify JointVarMap contains expected joint variables (4 revolute joints)
fprintf('\n2. Checking JointVarMap ... ');
jvKeys = keys(eSym.JointVarMap);
fprintf('%d joint variables: %s\n', ...
    numel(jvKeys), strjoin(jvKeys, ', '));
assert(numel(jvKeys) == 4, 'Expected 4 joint variables (4-bar linkage).');
assert(isKey(eSym.JointVarMap, 'joint_AB.q'), 'joint_AB.q missing.');
assert(isKey(eSym.JointVarMap, 'joint_BD.q'), 'joint_BD.q missing.');
assert(isKey(eSym.JointVarMap, 'joint_CD.q'), 'joint_CD.q missing.');
assert(isKey(eSym.JointVarMap, 'joint_AC.q'), 'joint_AC.q missing.');

%% 3. Run KinematicModel to extract end-frame symbolic pose
%   Link A (A1) is grounded; D2 is reachable via tree edges:
%   A1 → joint_AB → B1 → B2 → joint_BD → D1 → D2.
%   The closed chord (joint_AC → A3) is excluded from FK propagation.
endFrame = 'frame_link_D2.frame_hyper_cube';
fprintf('\n3. Running KinematicModel to endFrame="%s" ... ', endFrame);
tf = solver.KinematicModel(eSym.EdgeGraph_, endFrame, eSym.JointVarMap);
fprintf('OK\n');
fprintf('   JointVars on FK path: %s\n', strjoin(string(tf.JointVars), ', '));
fprintf('   TSym type: %s, size: %dx%d\n', class(tf.TSym), size(tf.TSym));

%% 4. Verify TSym is symbolic (structural checks)
fprintf('\n4. Verifying TSym structure ... ');
assert(isa(tf.TSym, 'sym'), 'TSym must be sym type.');
ts = char(tf.TSym);
assert(contains(ts, 'cos') || contains(ts, 'sin'), ...
    'TSym should contain trig terms from revolute joints.');
% Parallelogram linkage: TSym should depend on joint_AB, joint_BD, joint_CD
% (tree path A1→B1→D1→C2). joint_AC only appears in the closed chord.
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

% Evaluate at joint_config values (default all-zero pose) as numeric sanity check.
% Use JointVarMap (canonical name → sym) to map config values by name,
% eliminating the fragile symvar alphabetical-order dependency.
jointCfg = core.readYaml(configFile);
vMap = containers.Map();
instNames = fieldnames(jointCfg);
for ii = 1:numel(instNames)
    iname = instNames{ii};
    ov = jointCfg.(iname);
    if ~isstruct(ov); continue; end
    varNames = fieldnames(ov);
    for jj = 1:numel(varNames)
        canonicalName = [iname '.' varNames{jj}];
        if isKey(eSym.JointVarMap, canonicalName)
            vMap(canonicalName) = ov.(varNames{jj});
        end
    end
end

T_num = tf.eval(vMap);
p_num = tf.evalPos(vMap);
R_num = tf.evalRot(vMap);

fprintf('OK\n');
fprintf('   Joint values (all zero pose, straight-line configuration):\n');
vKeys = keys(vMap);
for i = 1:numel(vKeys)
    fprintf('     %s = %.4f\n', vKeys{i}, vMap(vKeys{i}));
end
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
