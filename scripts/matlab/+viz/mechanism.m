function result = mechanism(dslYaml, configYaml)
%MECHANISM  Validate & visualize a full modular mechanism assembly.
%   MECHANISM(DSLYAML) parses a mechanism-assembly DSL file from
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
%   MECHANISM(DSLYAML, CONFIGYAML) also injects per-instance joint
%   variable values (revolute q, prismatic dx/dy/dz) from a config YAML keyed
%   by instance name -> variable. Unlisted joint variables default to 0
%   (zero pose). Geometric module parameters (cubeLength, tipDistance, ...)
%   still come from <module_library>/config/dimensions.yaml keyed by
%   module_type.
%
%   RESULT = MECHANISM(...) returns a struct with the mechanism
%   name, the computed global pose map (frame name -> 4x4), and the list of
%   any frames that could not be placed, useful for headless checking.
%
%   Example:
%     viz.mechanism('../../specs/dsl/examples/open-chain-2r/open-chain-2r.yaml', ...
%                  '../../specs/dsl/examples/open-chain-2r/joint_config.yaml')
%

    if nargin < 1 || isempty(dslYaml)
        error('viz:mechanism:usage', ...
            'Usage: viz.mechanism(dslYaml[, configYaml])');
    end
    if nargin < 2; configYaml = ''; end

    here = fileparts(fileparts(mfilename('fullpath')));
    repoRoot = fileparts(fileparts(here));

    dslYaml = core.PathUtils.resolve(dslYaml, here);
    assert(exist(dslYaml, 'file') > 0, 'DSL file not found: %s', dslYaml);

    dsl = core.readYaml(dslYaml);
    assert(isfield(dsl, 'mechanism'), 'Not a mechanism assembly: %s', dslYaml);

    % validate DSL version (currently only v0 is supported)
    ver = core.CommonUtils.field(dsl, 'dsl_version', 0);
    assert(isequal(ver, 0), 'Unsupported dsl_version %s (expected 0).', num2str(ver));

    mechName = dsl.mechanism;
    dslDir = fileparts(dslYaml);

    libRel = core.CommonUtils.field(dsl, 'module_library', '../../modules/');

    % module library absolute path
    libDir = core.PathUtils.resolve(libRel, dslDir);
    assert(exist(libDir, 'dir') > 0, 'Module library not found: %s', libRel);

    % load the module library's geometric parameter config (by module_type)
    dimCfg = struct();
    dimCfgPath = fullfile(libDir, 'config', 'dimensions.yaml');
    if exist(dimCfgPath, 'file'); dimCfg = core.readYaml(dimCfgPath); end

    % load per-instance joint variable overrides from the mechanism config
    jointCfg = struct();
    if ~isempty(configYaml)
        configYaml = core.PathUtils.resolve(configYaml, here);
        if exist(configYaml, 'file'); jointCfg = core.readYaml(configYaml); end
    end

    assert(isfield(dsl, 'instances') && isstruct(dsl.instances), ...
        'Mechanism %s declares no instances.', mechName);
    instNames = fieldnames(dsl.instances);

    % number of instances in this mechanism
    nInst = numel(instNames);
    
    % shared pose-graph accumulator: edges, ground nodes
    g = ir.EdgeGraph();
    
    % cache module definitions by type to avoid re-reading the same YAML file
    defCache = containers.Map('KeyType', 'char', 'ValueType', 'any');

    % Instance struct array: name, type, module def, bodies, frames, joints
    inst = struct('name', {}, 'type', {}, 'md', {}, 'bodies', {}, ...
        'frames', {}, 'joints', {});

    for i = 1:nInst
        iname = instNames{i};
        itype = dsl.instances.(iname).type;

        % expand this instance: load module def, inject params, expand bodies/frames/edges
        inst(i) = localExpandInstance(g, iname, itype, libDir, defCache, dimCfg, jointCfg);
    end

    % --- connections: mate socket->plug, insert bidirectional mate edges ---
    conns = core.CommonUtils.asList(core.CommonUtils.field(dsl, 'connections', {}));
    connInfo = struct('socketNode', {}, 'plugNode', {}, 'closed', {}, 'label', {});
    for c = 1:numel(conns)
        cn = conns{c};
        assert(isfield(cn, 'ports') && numel(cn.ports) == 2, ...
            'Connection %d must list exactly two ports.', c);
        refA = cn.ports{1}; refB = cn.ports{2};
        [ia, pa] = localParsePort(refA);
        [ib, pb] = localParsePort(refB);
        fa = localLookupFrame(inst, ia, pa);
        fb = localLookupFrame(inst, ib, pb);
        assert(~isempty(fa), 'Connection %d: unknown port "%s".', c, refA);
        assert(~isempty(fb), 'Connection %d: unknown port "%s".', c, refB);

        % check polarity: one socket + one plug
        polA = fa.polarity; polB = fb.polarity;
        if strcmp(polA, 'socket') && strcmp(polB, 'plug')
            sk = fa; pl = fb;
        elseif strcmp(polA, 'plug') && strcmp(polB, 'socket')
            sk = fb; pl = fa;
        else
            error('viz:mechanism:polarity', ...
                ['Connection %d [%s ~ %s]: expected one socket + one plug, ' ...
                 'got "%s" + "%s".'], c, refA, refB, core.CommonUtils.tern(isempty(polA), ...
                 'none', polA), core.CommonUtils.tern(isempty(polB), 'none', polB));
        end

        % connection parameters: roll (deg) and symmetry (integer)
        roll = core.CommonUtils.field(cn, 'roll', 0);
        sym = core.CommonUtils.field(sk, 'symmetry', 4);

        isClosed = isequal(core.CommonUtils.field(cn, 'closed', false), true);

        % Only spanning-tree (non-chord) mates propagate poses. A chord edge
        % (closed:true) is the cut of a kinematic loop; using it for
        % propagation would place a frame through the loop and leave the
        % closure residual on a tree edge (and make bodies reached from
        % multiple loops inconsistent). Chords stay diagnostic-only, so their
        % mate gap correctly reports the loop-closure residual at this config.
        if ~isClosed
            g.addMate(sk.node, pl.node, roll, sym);
        else
            g.addClosedMate(sk.node, pl.node, roll, sym);
        end

        connInfo(end+1) = struct('socketNode', sk.node, 'plugNode', pl.node, ...
            'closed', isClosed, ...
            'label', sprintf('%s~%s', sk.node, pl.node)); %#ok<AGROW>
    end

    % --- world root(s) & FK propagation ---
    if ~g.hasGroundNodes()
        g.addGround(inst(1).bodies{1}.node);   % first body of first instance
    end

    % poses is a containers.Map of frame name -> 4x4 homogeneous transform
    poses = g.propagate();

    % --- characteristic scale ---
    maxr = 1; ks = keys(poses);
    % compute the maximum distance from the origin to all frames
    for k = 1:numel(ks); P = poses(ks{k}); maxr = max(maxr, norm(P(1:3, 4))); end
    % L, the characteristic length, is used to control the size of the triads and joint axes in the visualization
    L = max(4, 0.20 * maxr);

    % --- figure ---
    fig = figure('Name', sprintf('Mechanism: %s', mechName), 'Color', 'w');
    ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    view(ax, 135, 25); xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');
    title(ax, sprintf('%s  —  %d instances, %d connections  (X=red Y=green Z=blue)', ...
        mechName, nInst, numel(conns)), 'Interpreter', 'none');
    
    % draw the world frame triad at the origin
    core.VizHelpers.triad(ax, eye(4), L * 1.4, 2.5, '-');
    text(ax, 0, 0, 0, '  world', 'FontWeight', 'bold', 'Color', [.2 .2 .2]);

    % --- bodies + frames per instance ---
    result.mechanism = mechName;
    result.poses = poses;
    unplaced = {};
    fprintf('\n=== mechanism: %s (%d instances, %d connections) ===\n', ...
        mechName, nInst, numel(conns));

    for i = 1:nInst
        col = core.VizHelpers.typeColor(inst(i).type);
        fprintf('\n-- instance %s [%s] --\n', inst(i).name, inst(i).type);

        for k = 1:numel(inst(i).bodies)
            b = inst(i).bodies{k};
            % check if the body node has a computed pose; if not, mark it as unplaced
            if ~isKey(poses, b.node); unplaced{end+1} = b.node; continue; end %#ok<AGROW>

            Tb = poses(b.node);
            geomPath = core.PathUtils.resolveGeometryPath(b.geometry, libDir, repoRoot);
            if ~isempty(b.geometry) && ~isempty(geomPath)
                geom = core.VizHelpers.importGeometry(geomPath);
                if ~isempty(geom); core.VizHelpers.patchGeometry(ax, Tb, geom, col, 0.12); end
            end
            % draw the body frame triad and label
            core.VizHelpers.triad(ax, Tb, L, 1.2, '-');
        end

        for k = 1:numel(inst(i).frames)
            f = inst(i).frames{k};
            if ~isKey(poses, f.node)
                fprintf('  [UNPLACED] %-22s\n', f.node);
                unplaced{end+1} = f.node; %#ok<AGROW>
                continue;
            end
            T = poses(f.node);
            if f.exposed
                lw = 2.0; sty = '-'; mk = 'PORT';
            else
                lw = 1.0; sty = '--'; mk = 'frame';
            end

            % draw the frame triad and label
            core.VizHelpers.triad(ax, T, L, lw, sty);

            core.VizHelpers.frameMarker(ax, T, f.node, f.exposed);
            fprintf('  %-7s %-22s pos=[% 7.2f % 7.2f % 7.2f]  +Z=[% .2f % .2f % .2f]\n', ...
                mk, f.node, T(1,4), T(2,4), T(3,4), T(1,3), T(2,3), T(3,3));
        end

        % joint axes
        for k = 1:numel(inst(i).joints)
            j = inst(i).joints{k};
            if ~isKey(poses, j.node); continue; end
            core.VizHelpers.jointAxis(ax, poses(j.node), j.axis, L, j.kind, ...
                sprintf('%s.%s=%.3g', inst(i).name, j.var, j.val));
        end
    end

    % --- mate diagnostics: segment between paired port origins ---
    fprintf('\n-- mate checks --\n');
    for c = 1:numel(connInfo)
        ci = connInfo(c);
        % check if both socket and plug nodes have computed poses; if not, mark them as unplaced
        if ~isKey(poses, ci.socketNode) || ~isKey(poses, ci.plugNode)
            fprintf('  [UNPLACED MATE] %s\n', ci.label); continue;
        end
        Ps = poses(ci.socketNode); Pp = poses(ci.plugNode);

        % check if the origins of the socket and plug are aligned; if not, draw a dashed line between them
        gap = norm(Ps(1:3,4) - Pp(1:3,4));
        zdot = dot(Ps(1:3,3), Pp(1:3,3));   % +Z should be anti-parallel (-1)

        % draw a dashed line between the socket and plug origins, colored by whether the mate is closed or not
        if ci.closed; lc = [0.95 0.55 0.10]; lw = 3.0; else; lc = [0.2 0.2 0.2]; lw = 1.5; end
        line(ax, [Ps(1,4) Pp(1,4)], [Ps(2,4) Pp(2,4)], [Ps(3,4) Pp(3,4)], ...
            'Color', lc, 'LineWidth', lw, 'LineStyle', '--');
        fprintf('  %-40s gap=%.3e  Zdot=% .4f%s\n', ci.label, gap, zdot, ...
            core.CommonUtils.tern(ci.closed, '  [closed]', ''));
    end

    if ~isempty(unplaced)
        fprintf('\n  [WARNING] %d node(s) not placed (disconnected component?).\n', ...
            numel(unplaced));
    end
    result.unplaced = unplaced;

    rotate3d(ax, 'on');
end

%% expand a single instance: load module def, inject params, expand bodies/frames/edges
function inst_i = localExpandInstance(g, iname, itype, ...
        libDir, defCache, dimCfg, jointCfg)
    if isKey(defCache, itype)
        % md stands for "module definition" (the parsed YAML struct for this module type)
        md = defCache(itype);
    else
        fp = fullfile(libDir, [itype '.yaml']);
        assert(exist(fp, 'file') > 0, ...
            'Instance "%s": module type "%s" not found in %s.', iname, itype, libDir);
        md = core.readYaml(fp);

        % cache the module definition for future instances of this type
        defCache(itype) = md;
    end

    % inject geometric parameters (cubeLength, tipDistance, ...) from the module library's config/dimensions.yaml
    params = struct();
    if isfield(dimCfg, itype)
        params = dimCfg.(itype);
    end

    if isfield(jointCfg, iname)
        % ov stands for "overrides" (per-instance joint variable values from the mechanism config)
        ov = jointCfg.(iname);
        if isstruct(ov)
            % ofn stands for "override field names" (the joint variable names to override)
            ofn = fieldnames(ov);
            for q = 1:numel(ofn); params.(ofn{q}) = ov.(ofn{q}); end
        end
    end

    % inject the instance name as a prefix to each node name, for example "inst1.body" or "inst2.faceXPlus"
    pre = [iname '.'];

    % expand bodies
    bodies = core.CommonUtils.asList(core.CommonUtils.field(md, 'bodies', {}));
    bList = cell(1, numel(bodies));
    for k = 1:numel(bodies)
        b = bodies{k};
        % assemble the body struct with the prefixed node name, original name, and geometry path (if any)
        bList{k} = struct('node', [pre b.name], 'name', b.name, ...
            'geometry', core.CommonUtils.field(b, 'geometry', ''));
    end

    % expand frames
    frames = core.CommonUtils.asList(core.CommonUtils.field(md, 'frames', {}));
    fList = cell(1, numel(frames));
    for k = 1:numel(frames)
        f = frames{k};
        node = [pre f.name];

        % assemble the frame struct with the prefixed node name, original name, exposed flag, polarity, semantic tag, and symmetry
        fList{k} = struct('node', node, 'name', f.name, ...
            'exposed', isfield(f, 'exposed') && isequal(f.exposed, true), ...
            'polarity', core.CommonUtils.field(f, 'polarity', ''), ...
            'semantic_tag', core.CommonUtils.field(f, 'semantic_tag', ''), ...
            'symmetry', core.CommonUtils.field(f, 'symmetry', 4));

        % if this frame is marked as a ground frame, add it to the pose graph's ground nodes
        if strcmp(core.CommonUtils.field(f, 'semantic_tag', ''), 'ground')
            g.addGround(node);
        end
    end

    % expand fixed transforms
    % fixed transforms are always fully determined — no pending rotation placeholder needed here
    fts = core.CommonUtils.asList(core.CommonUtils.field(md, 'fixed_transforms', {}));
    for k = 1:numel(fts)
        t = fts{k};
        tr = core.CommonUtils.evalVec(t.translation, params);
        R = core.RigidBodyMath.rot(t.rotation, params);

        % assemble the 4x4 homogeneous transform matrix from rotation R and translation tr
        T = core.RigidBodyMath.T(R, tr);
        g.addFixedTransform([pre t.from_frame], [pre t.to_frame], T);
    end

    % expand joints
    jts = core.CommonUtils.asList(core.CommonUtils.field(md, 'joints', {}));
    jList = cell(1, numel(jts));
    for k = 1:numel(jts)
        j = jts{k};
        ax = core.CommonUtils.evalVec(j.axis, params);
        val = core.CommonUtils.field(params, j.variable, 0);
        kind = core.CommonUtils.field(j, 'kind', 'revolute');
        g.addJoint([pre j.from_frame], [pre j.to_frame], ax, val, kind);

        % assemble the joint struct with the prefixed node name, axis, variable name, value, and kind
        jList{k} = struct('node', [pre j.from_frame], 'axis', ax(:) / max(norm(ax), eps), ...
            'var', j.variable, 'val', val, 'kind', kind);
    end

    % assemble the instance struct with the name, type, module definition, and lists of bodies, frames, and joints
    inst_i = struct('name', iname, 'type', itype, 'md', md, ...
        'bodies', {bList}, 'frames', {fList}, 'joints', {jList});
end

%% split an "instance.port" reference into its two parts (port may lack a dot only if malformed)
function [instName, portName] = localParsePort(ref)
    d = strfind(ref, '.');
    assert(~isempty(d), 'Malformed port reference "%s" (expected instance.port).', ref);
    instName = ref(1:d(1)-1);
    portName = ref(d(1)+1:end);
end

%% find the recorded frame struct for instance/port; [] if absent
function f = localLookupFrame(inst, instName, portName)
    f = [];
    for i = 1:numel(inst)
        if ~strcmp(inst(i).name, instName); continue; end
        for k = 1:numel(inst(i).frames)
            if strcmp(inst(i).frames{k}.name, portName); f = inst(i).frames{k}; return; end
        end
    end
end




