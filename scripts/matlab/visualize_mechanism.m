function result = visualize_mechanism(dslYaml, configYaml)
%VISUALIZE_MECHANISM  Validate & visualize a full modular mechanism assembly.
%   VISUALIZE_MECHANISM(DSLYAML) parses a mechanism-assembly DSL file from
%   specs/dsl/examples/*.yaml, loads every referenced L1 module definition
%   from the module library, expands each instance's internal frame graph,
%   mates instances port-to-port per the connections list, propagates global
%   poses via forward kinematics, and draws:
%     - each body as imported STEP geometry when available (colored by type)
%     - each frame/port as an RGB coordinate triad (X=red, Y=green, Z=blue)
%     - each mate as a diagnostic segment between the paired port origins
%       (zero-length when aligned; a visible gap reveals mis-mated ports)
%     - each 1-DOF joint axis as a highlighted segment
%
%   VISUALIZE_MECHANISM(DSLYAML, CONFIGYAML) also injects per-instance joint
%   variable values (revolute q, prismatic dx/dy/dz) from a config YAML keyed
%   by mechanism name -> instance name -> variable. Unlisted joint variables
%   default to 0 (zero pose). Geometric module parameters (cubeLength,
%   tipDistance, ...) still come from <module_library>/config/parameters.yaml
%   keyed by module_type.
%
%   RESULT = VISUALIZE_MECHANISM(...) returns a struct with the mechanism
%   name, the computed global pose map (frame name -> 4x4), and the list of
%   any frames that could not be placed, useful for headless checking.
%
%   Example:
%     visualize_mechanism('../../specs/dsl/examples/open-chain-2r.yaml', ...
%                         'mechanism_viz_config.yaml')
%
%   Shared primitives (rotation/geometry/FK) live in scripts/matlab/+smk/
%   and are reused verbatim from visualize_module.m. Mate convention follows
%   specs/modeling-conventions.md and specs/dsl/connection-semantics.md:
%     T_plug<-socket = Rz(roll*2*pi/symmetry) * Rx(pi),  t = 0.
%   socket is the parent (Frame face / Manipulator dock); plug is the child.

    if nargin < 1 || isempty(dslYaml)
        error('visualize_mechanism:usage', ...
            'Usage: visualize_mechanism(dslYaml[, configYaml])');
    end
    if nargin < 2; configYaml = ''; end

    here = fileparts(mfilename('fullpath'));

    % dslYaml may be relative to this script; resolve to absolute path
    dslYaml = smk.PathUtils.resolve(dslYaml, here);
    assert(exist(dslYaml, 'file') > 0, 'DSL file not found: %s', dslYaml);

    % parse dslYaml into a struct; validate top-level fields
    dsl = read_module_yaml(dslYaml);
    assert(isfield(dsl, 'mechanism'), 'Not a mechanism assembly: %s', dslYaml);
    ver = smk.CommonUtils.field(dsl, 'dsl_version', 0);
    assert(isequal(ver, 0), 'Unsupported dsl_version %s (expected 0).', num2str(ver));

    mechName = dsl.mechanism;
    dslDir = fileparts(dslYaml);

    % find the module library relative to the DSL file
    libRel = smk.CommonUtils.field(dsl, 'module_library', '../../modules/');

    % resolve to absolute path
    libDir = smk.PathUtils.resolve(libRel, dslDir);
    assert(exist(libDir, 'dir') > 0, 'Module library not found: %s', libRel);

    % module_type -> file path
    typeIndex = local_module_index(libDir);

    % load the module library's geometric parameter config (by module_type)
    paramCfg = struct();
    paramCfgPath = fullfile(libDir, 'config', 'parameters.yaml');
    if exist(paramCfgPath, 'file'); paramCfg = read_module_yaml(paramCfgPath); end

    % load the mechanism config (by instance name) if provided
    mechCfg = struct();
    if ~isempty(configYaml)
        configYaml = smk.PathUtils.resolve(configYaml, here);
        if exist(configYaml, 'file'); mechCfg = read_module_yaml(configYaml); end
    end

    % per-instance joint overrides for this mechanism
    mechOv = struct();
    mnf = matlab.lang.makeValidName(mechName);
    if isfield(mechCfg, mnf) && isstruct(mechCfg.(mnf)); mechOv = mechCfg.(mnf); end

    % --- iterate instances: load defs, inject params, expand internal graph ---
    assert(isfield(dsl, 'instances') && isstruct(dsl.instances), ...
        'Mechanism %s declares no instances.', mechName);

    instNames = fieldnames(dsl.instances);

    % number of instances in this mechanism
    nInst = numel(instNames);
    
    % store all internal fixed-transform and joint edges, plus inter-instance mates
    edges = struct('from', {}, 'to', {}, 'T', {});

    % record pending frames (from fixed transforms) for diagnostic triad coloring
    pendingMap = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    % cache module definitions by type to avoid re-reading the same YAML file
    defCache = containers.Map('KeyType', 'char', 'ValueType', 'any');

    % initialize instance structures
    inst = struct('name', {}, 'type', {}, 'md', {}, 'bodies', {}, ...
        'frames', {}, 'joints', {});
    
    % record all frames marked as "ground" for seeding the FK pose propagation
    groundNodes = {};

    % iterate instances: load defs, inject params, expand internal graph
    for i = 1:nInst
        iname = instNames{i};

        % instance type is the module_type of the referenced module definition
        itype = dsl.instances.(iname).type;

        if isKey(defCache, itype)
            md = defCache(itype);
        else
            assert(isKey(typeIndex, itype), ...
                'Instance "%s": module type "%s" not found in library %s.', ...
                iname, itype, libRel);

            % read module definition YAML file
            md = read_module_yaml(typeIndex(itype));

            % cache the module definition for future instances of this type
            defCache(itype) = md;
        end

        % params = geometric params (by module_type) + joint overrides (by instance)
        params = struct();
        tf = matlab.lang.makeValidName(itype);
        if isfield(paramCfg, tf) && isstruct(paramCfg.(tf)); params = paramCfg.(tf); end

        % joint overrides for this instance (from the mechanism config)    
        inf = matlab.lang.makeValidName(iname);

        % check for joint variable overrides in the mechanism config for this instance
        if isfield(mechOv, inf) && isstruct(mechOv.(inf))
            % ov: joint variable overrides for this instance; ofn: joint variable names
            ov = mechOv.(inf); ofn = fieldnames(ov);

            % override any joint variables in the module definition with the mechanism config
            for q = 1:numel(ofn); params.(ofn{q}) = ov.(ofn{q}); end
        end

        % prefix for this instance
        pre = [iname '.'];

        % bodies
        bodies = smk.CommonUtils.aslist(smk.CommonUtils.field(md, 'bodies', {}));
        bList = cell(1, numel(bodies));
        for k = 1:numel(bodies)
            b = bodies{k};
            bList{k} = struct('node', [pre b.name], 'name', b.name, ...
                'geometry', smk.CommonUtils.field(b, 'geometry', ''));
        end

        % frames (record polarity/exposed/tag for mating + rendering)
        frames = smk.CommonUtils.aslist(smk.CommonUtils.field(md, 'frames', {}));
        fList = cell(1, numel(frames));
        for k = 1:numel(frames)
            f = frames{k};
            node = [pre f.name];
            fList{k} = struct('node', node, 'name', f.name, ...
                'exposed', isfield(f, 'exposed') && isequal(f.exposed, true), ...
                'polarity', smk.CommonUtils.field(f, 'polarity', ''), ...
                'semantic_tag', smk.CommonUtils.field(f, 'semantic_tag', ''), ...
                'symmetry', smk.CommonUtils.field(f, 'symmetry', 4));
            if strcmp(smk.CommonUtils.field(f, 'semantic_tag', ''), 'ground')
                groundNodes{end+1} = node; %#ok<AGROW>
            end
        end

        % internal fixed-transform edges (bidirectional)
        fts = smk.CommonUtils.aslist(smk.CommonUtils.field(md, 'fixed_transforms', {}));
        for k = 1:numel(fts)
            t = fts{k};
            tr = smk.CommonUtils.eval_vec(t.translation, params);
            [R, pend] = smk.RigidBodyMath.rot(t.rotation, params);
            T = smk.RigidBodyMath.T(R, tr);
            fromN = [pre t.from_frame]; toN = [pre t.to_frame];
            edges(end+1) = struct('from', fromN, 'to', toN, 'T', T); %#ok<AGROW>
            edges(end+1) = struct('from', toN, 'to', fromN, 'T', local_invT(T)); %#ok<AGROW>
            if pend; pendingMap(toN) = true; end
        end

        % internal joint edges (kind-aware, bidirectional)
        jts = smk.CommonUtils.aslist(smk.CommonUtils.field(md, 'joints', {}));
        jList = cell(1, numel(jts));
        for k = 1:numel(jts)
            j = jts{k};
            ax = smk.CommonUtils.eval_vec(j.axis, params);
            val = smk.CommonUtils.field(params, j.variable, 0);
            kind = smk.CommonUtils.field(j, 'kind', 'revolute');
            T = smk.PoseGraph.joint_transform(kind, ax, val);
            fromN = [pre j.from_frame]; toN = [pre j.to_frame];
            edges(end+1) = struct('from', fromN, 'to', toN, 'T', T); %#ok<AGROW>
            edges(end+1) = struct('from', toN, 'to', fromN, 'T', local_invT(T)); %#ok<AGROW>
            jList{k} = struct('node', fromN, 'axis', ax(:) / max(norm(ax), eps), ...
                'var', j.variable, 'val', val, 'kind', kind);
        end

        inst(i) = struct('name', iname, 'type', itype, 'md', md, ...
            'bodies', {bList}, 'frames', {fList}, 'joints', {jList});
    end

    % --- connections: mate socket->plug, insert bidirectional mate edges ---
    conns = smk.CommonUtils.aslist(smk.CommonUtils.field(dsl, 'connections', {}));
    connInfo = struct('socketNode', {}, 'plugNode', {}, 'closed', {}, 'label', {});
    for c = 1:numel(conns)
        cn = conns{c};
        assert(isfield(cn, 'ports') && numel(cn.ports) == 2, ...
            'Connection %d must list exactly two ports.', c);
        refA = cn.ports{1}; refB = cn.ports{2};
        [ia, pa] = local_parse_port(refA);
        [ib, pb] = local_parse_port(refB);
        fa = local_lookup_frame(inst, ia, pa);
        fb = local_lookup_frame(inst, ib, pb);
        assert(~isempty(fa), 'Connection %d: unknown port "%s".', c, refA);
        assert(~isempty(fb), 'Connection %d: unknown port "%s".', c, refB);

        polA = fa.polarity; polB = fb.polarity;
        if strcmp(polA, 'socket') && strcmp(polB, 'plug')
            sk = fa; pl = fb;
        elseif strcmp(polA, 'plug') && strcmp(polB, 'socket')
            sk = fb; pl = fa;
        else
            error('visualize_mechanism:polarity', ...
                ['Connection %d [%s ~ %s]: expected one socket + one plug, ' ...
                 'got "%s" + "%s".'], c, refA, refB, smk.CommonUtils.tern(isempty(polA), ...
                 'none', polA), smk.CommonUtils.tern(isempty(polB), 'none', polB));
        end

        roll = smk.CommonUtils.field(cn, 'roll', 0);
        sym = smk.CommonUtils.field(sk, 'symmetry', 4);
        rollAngle = roll * 2 * pi / sym;
        Tm = smk.RigidBodyMath.T(smk.RigidBodyMath.rotz(rollAngle) * smk.RigidBodyMath.rotx(pi), [0; 0; 0]);
        isClosed = isequal(smk.CommonUtils.field(cn, 'closed', false), true);

        % Only spanning-tree (non-chord) mates propagate poses. A chord edge
        % (closed:true) is the cut of a kinematic loop; using it for
        % propagation would place a frame through the loop and leave the
        % closure residual on a tree edge (and make bodies reached from
        % multiple loops inconsistent). Chords stay diagnostic-only, so their
        % mate gap correctly reports the loop-closure residual at this config.
        if ~isClosed
            edges(end+1) = struct('from', sk.node, 'to', pl.node, 'T', Tm); %#ok<AGROW>
            edges(end+1) = struct('from', pl.node, 'to', sk.node, 'T', local_invT(Tm)); %#ok<AGROW>
        end

        connInfo(end+1) = struct('socketNode', sk.node, 'plugNode', pl.node, ...
            'closed', isClosed, ...
            'label', sprintf('%s~%s', sk.node, pl.node)); %#ok<AGROW>
    end

    % --- world root(s) & FK propagation ---
    seed = containers.Map('KeyType', 'char', 'ValueType', 'any');
    if ~isempty(groundNodes)
        for k = 1:numel(groundNodes); seed(groundNodes{k}) = eye(4); end
    else
        seed(inst(1).bodies{1}.node) = eye(4);   % first body of first instance
    end
    poses = smk.PoseGraph.propagate_poses(edges, seed);

    % --- characteristic scale ---
    maxr = 1; ks = keys(poses);
    for k = 1:numel(ks); P = poses(ks{k}); maxr = max(maxr, norm(P(1:3, 4))); end
    L = max(4, 0.20 * maxr);

    moduleDir = libDir;
    repoRoot = fileparts(fileparts(here));

    % --- figure ---
    fig = figure('Name', sprintf('Mechanism: %s', mechName), 'Color', 'w');
    ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    view(ax, 135, 25); xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');
    title(ax, sprintf('%s  —  %d instances, %d connections  (X=red Y=green Z=blue)', ...
        mechName, nInst, numel(conns)), 'Interpreter', 'none');

    smk.VizHelpers.triad(ax, eye(4), L * 1.4, 2.5, '-');
    text(ax, 0, 0, 0, '  world', 'FontWeight', 'bold', 'Color', [.2 .2 .2]);

    % --- bodies + frames per instance ---
    result.mechanism = mechName;
    result.poses = poses;
    unplaced = {};
    fprintf('\n=== mechanism: %s (%d instances, %d connections) ===\n', ...
        mechName, nInst, numel(conns));

    for i = 1:nInst
        col = smk.VizHelpers.type_color(inst(i).type);
        fprintf('\n-- instance %s [%s] --\n', inst(i).name, inst(i).type);

        for k = 1:numel(inst(i).bodies)
            b = inst(i).bodies{k};
            if ~isKey(poses, b.node); unplaced{end+1} = b.node; continue; end %#ok<AGROW>
            Tb = poses(b.node);
            geomPath = smk.PathUtils.resolve_geometry_path(b.geometry, moduleDir, repoRoot);
            if ~isempty(b.geometry) && ~isempty(geomPath)
                geom = smk.VizHelpers.import_geometry(geomPath);
                if ~isempty(geom); smk.VizHelpers.patch_geometry(ax, Tb, geom, col, 0.12); end
            end
            smk.VizHelpers.triad(ax, Tb, L, 1.2, '-');
        end

        for k = 1:numel(inst(i).frames)
            f = inst(i).frames{k};
            if ~isKey(poses, f.node)
                fprintf('  [UNPLACED] %-22s\n', f.node);
                unplaced{end+1} = f.node; %#ok<AGROW>
                continue;
            end
            T = poses(f.node);
            pend = isKey(pendingMap, f.node);
            if pend
                fc = [1 0 1]; lw = 2.0; sty = '--'; mk = 'magenta';
            elseif f.exposed
                fc = [0 0 0]; lw = 2.0; sty = '-'; mk = 'PORT';
            else
                fc = [0.45 0.45 0.45]; lw = 1.0; sty = '--'; mk = 'frame';
            end
            smk.VizHelpers.triad(ax, T, L, lw, sty);
            plot3(ax, T(1,4), T(2,4), T(3,4), 'o', 'MarkerSize', 5, ...
                'MarkerFaceColor', fc, 'MarkerEdgeColor', fc);
            lbl = f.node; if pend; lbl = [lbl ' (pending R)']; end %#ok<AGROW>
            text(ax, T(1,4), T(2,4), T(3,4), ['  ' lbl], 'Color', fc, ...
                'FontWeight', smk.CommonUtils.tern(f.exposed, 'bold', 'normal'), ...
                'Interpreter', 'none', 'FontSize', 8);
            fprintf('  %-7s %-22s pos=[% 7.2f % 7.2f % 7.2f]  +Z=[% .2f % .2f % .2f]%s\n', ...
                mk, f.node, T(1,4), T(2,4), T(3,4), T(1,3), T(2,3), T(3,3), ...
                smk.CommonUtils.tern(pend, '  <pending>', ''));
        end

        % joint axes
        for k = 1:numel(inst(i).joints)
            j = inst(i).joints{k};
            if ~isKey(poses, j.node); continue; end
            P = poses(j.node);
            o = P(1:3, 4); d = P(1:3, 1:3) * j.axis;
            if strcmpi(j.kind, 'prismatic'); jc = [0.1 0.1 0.9]; else; jc = [0.9 0.1 0.1]; end
            p1 = o - d * L * 1.3; p2 = o + d * L * 1.3;
            line(ax, [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
                'Color', jc, 'LineWidth', 2.5, 'LineStyle', ':');
            text(ax, p2(1), p2(2), p2(3), sprintf('  %s.%s=%.3g', ...
                inst(i).name, j.var, j.val), 'Color', jc, 'FontSize', 8, ...
                'Interpreter', 'none');
        end
    end

    % --- mate diagnostics: segment between paired port origins ---
    fprintf('\n-- mate checks --\n');
    for c = 1:numel(connInfo)
        ci = connInfo(c);
        if ~isKey(poses, ci.socketNode) || ~isKey(poses, ci.plugNode)
            fprintf('  [UNPLACED MATE] %s\n', ci.label); continue;
        end
        Ps = poses(ci.socketNode); Pp = poses(ci.plugNode);
        gap = norm(Ps(1:3,4) - Pp(1:3,4));
        zdot = dot(Ps(1:3,3), Pp(1:3,3));   % +Z should be anti-parallel (-1)
        if ci.closed; lc = [0.95 0.55 0.10]; lw = 3.0; else; lc = [0.2 0.2 0.2]; lw = 1.5; end
        line(ax, [Ps(1,4) Pp(1,4)], [Ps(2,4) Pp(2,4)], [Ps(3,4) Pp(3,4)], ...
            'Color', lc, 'LineWidth', lw, 'LineStyle', '--');
        fprintf('  %-40s gap=%.3e  Zdot=% .4f%s\n', ci.label, gap, zdot, ...
            smk.CommonUtils.tern(ci.closed, '  [closed]', ''));
    end

    if ~isempty(unplaced)
        fprintf('\n  [WARNING] %d node(s) not placed (disconnected component?).\n', ...
            numel(unplaced));
    end
    result.unplaced = unplaced;

    rotate3d(ax, 'on');
end

%% ---- mechanism-orchestration local functions (shared math lives in +smk/) ----

%% index the module library: module_type -> definition file path
% scan the module library for *.yaml files, parse each, and return a map of
% module_type -> absolute file path. Skip any files that fail to parse.
function idx = local_module_index(libDir)
    % initialize an empty map
    idx = containers.Map('KeyType', 'char', 'ValueType', 'char');
    files = dir(fullfile(libDir, '*.yaml'));

    for i = 1:numel(files)
        % fp is the absolute path to the module definition file
        fp = fullfile(libDir, files(i).name);
        try
            md = read_module_yaml(fp);
        catch
            continue;
        end
        if isstruct(md) && isfield(md, 'module_type')
            idx(md.module_type) = fp;
        end
    end
end

%% split an "instance.port" reference into its two parts (port may lack a dot only if malformed)
function [instName, portName] = local_parse_port(ref)
    d = strfind(ref, '.');
    assert(~isempty(d), 'Malformed port reference "%s" (expected instance.port).', ref);
    instName = ref(1:d(1)-1);
    portName = ref(d(1)+1:end);
end

%% find the recorded frame struct for instance/port; [] if absent
function f = local_lookup_frame(inst, instName, portName)
    f = [];
    for i = 1:numel(inst)
        if ~strcmp(inst(i).name, instName); continue; end
        for k = 1:numel(inst(i).frames)
            if strcmp(inst(i).frames{k}.name, portName); f = inst(i).frames{k}; return; end
        end
    end
end

%% inverse of a homogeneous (rigid) transform
function Ti = local_invT(T)
    R = T(1:3,1:3); t = T(1:3,4);
    Ti = eye(4); Ti(1:3,1:3) = R'; Ti(1:3,4) = -R' * t;
end


