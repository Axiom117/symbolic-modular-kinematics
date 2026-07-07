function result = mechanism(dslYaml, configYaml)
%MECHANISM  Validate & visualize a full modular mechanism assembly.
%   MECHANISM(DSLYAML) parses a mechanism-assembly DSL file from
%   specs/dsl/examples/*.yaml, delegates DSL→IR expansion to ir.Expander
%   (pure symbolic pipeline), substitutes joint values from config, then
%   draws:
%     - each body as imported STEP geometry when available (colored by type)
%     - each frame/port as an RGB coordinate triad (X=red, Y=green, Z=blue)
%     - each mate as a diagnostic segment between the paired port origins
%       (zero-length when aligned; a visible gap reveals mis-mated ports)
%     - each 1-DOF joint axis as a highlighted segment
%
%   MECHANISM(DSLYAML, CONFIGYAML) loads per-instance joint variable
%   values (revolute q, prismatic dx/dy/dz) from a config YAML keyed
%   by instance name -> variable.  These values are substituted into the
%   symbolic poses at render time (via ir.Expander.evaluateNumeric).
%   Unlisted joint variables default to 0 (zero pose).  Geometric module
%   parameters (cubeLength, tipDistance, ...) come from
%   <module_library>/config/dimensions.yaml keyed by module_type.
%
%   Pipeline (A.4.0 pure symbolic):
%     DSL → ir.Expander (symbolic) → evaluateNumeric(config) → render
%
%   RESULT = MECHANISM(...) returns a struct with the mechanism
%   name, the computed global pose map (frame name -> 4x4), and the list of
%   any frames that could not be placed, useful for headless checking.
%
%   Example:
%     viz.mechanism('../../specs/dsl/examples/open-chain-2r/robot_description.yaml', ...
%                  '../../specs/dsl/examples/open-chain-2r/joint_config.yaml')
%
%   See also: +ir/Expander, +ir/EdgeGraph, +core/PosePropagator

    if nargin < 1 || isempty(dslYaml)
        error('viz:mechanism:usage', ...
            'Usage: viz.mechanism(dslYaml[, configYaml])');
    end
    if nargin < 2; configYaml = ''; end

    % ---- path setup ----
    here = fileparts(fileparts(mfilename('fullpath')));
    repoRoot = fileparts(fileparts(here));

    % ---- DSL → IR expansion ----
    expander = ir.Expander(dslYaml);

    % ---- numeric evaluation for rendering (substitute joint values from config) ----
    poses = expander.evaluateNumeric(configYaml);

    % ---- read expanded state into local variables ----
    mechName = expander.MechName;
    inst     = expander.Instances;
    connInfo = expander.ConnectionInfo;
    libDir   = expander.LibDir;
    nInst    = numel(inst);
    nConns   = numel(connInfo);

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
        mechName, nInst, nConns), 'Interpreter', 'none');
    
    % draw the world frame triad at the origin
    core.VizHelpers.triad(ax, eye(4), L * 1.4, 2.5, '-');
    text(ax, 0, 0, 0, '  world', 'FontWeight', 'bold', 'Color', [.2 .2 .2]);

    % --- bodies + frames per instance ---
    result.mechanism = mechName;
    result.poses = poses;
    unplaced = {};
    fprintf('\n=== mechanism: %s (%d instances, %d connections) ===\n', ...
        mechName, nInst, nConns);

    % --- iterate over instances and draw bodies, frames, and joints ---
    for i = 1:nInst
        col = core.VizHelpers.typeColor(inst(i).type);
        fprintf('\n-- instance %s [%s] --\n', inst(i).name, inst(i).type);

        % draw each body for this instance
        for k = 1:numel(inst(i).bodies)
            b = inst(i).bodies{k};
            % check if the body node has a computed pose; if not, mark it as unplaced
            if ~isKey(poses, b.node); unplaced{end+1} = b.node; continue; end %#ok<AGROW>

            % Tb: the 4x4 homogeneous transform of the body in world coordinates
            Tb = poses(b.node);
            
            geomPath = core.PathUtils.resolveGeometryPath(b.geometry, libDir, repoRoot);
            if ~isempty(b.geometry) && ~isempty(geomPath)
                geom = core.VizHelpers.importGeometry(geomPath);
                if ~isempty(geom); core.VizHelpers.patchGeometry(ax, Tb, geom, col, 1); end
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
            jKey = [inst(i).name '.' j.var];
            if isKey(expander.JointValues, jKey)
                jVal = expander.JointValues(jKey);
            else
                jVal = 0;
            end
            core.VizHelpers.jointAxis(ax, poses(j.node), j.axis, L, j.kind, ...
                sprintf('%s.%s=%.3g', inst(i).name, j.var, jVal));
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




