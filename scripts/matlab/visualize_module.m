function result = visualize_module(moduleYaml, configYaml)
%VISUALIZE_MODULE  Validate & visualize one L1 module definition.
%   VISUALIZE_MODULE(MODULEYAML) parses a module definition from
%   specs/modules/*.yaml, builds its internal frame graph (body center
%   frames + ports), evaluates all fixedTransform / joint edges, and draws:
%     - each body as imported STEP geometry when available
%     - each frame/port as an RGB coordinate triad (X=red, Y=green, Z=blue)
%
%   VISUALIZE_MODULE(MODULEYAML, CONFIGYAML) also injects numeric parameter
%   values (e.g. cubeLength, tipDistance) and joint variable values from a
%   config YAML keyed by module_type, so symbolic translations like
%   'cubeLength/2' and the revolute angle 'q' resolve to numbers.
%
%   RESULT = VISUALIZE_MODULE(...) returns a struct with the computed global
%   pose of every frame (4x4), useful for headless checking.
%
%   Example:
%     visualize_module('../../specs/modules/frame.yaml', 'module_viz_config.yaml')
%
%   Rotation conventions follow specs/modeling-conventions.md:
%     - align{a,b}: rule 's -> d' means child-frame axis 's' equals parent
%       axis 'd'; R_child_in_parent = DST*SRC' (third axis by right-hand rule).
%     - rpy = [Rx,Ry,Rz] intrinsic Z-Y-X: R = Rz*Ry*Rx.
%     - axis_angle = Rodrigues(omega, q).
%     - pending => identity rotation, flagged in magenta (value not yet frozen).

    if nargin < 1 || isempty(moduleYaml)
        error('visualize_module:usage', ...
            'Usage: visualize_module(moduleYaml[, configYaml])');
    end
    if nargin < 2; configYaml = ''; end

    % resolve a relative path against the current script's directory
    here = fileparts(mfilename('fullpath'));
    moduleYaml = local_resolve(moduleYaml, here);

    m = read_module_yaml(moduleYaml);

    % -- validate module definition ---
    assert(isfield(m, 'module_type'), 'Not a module definition: %s', moduleYaml);

    % --- parameter injection from config (keyed by module_type) ---
    params = struct();
    if ~isempty(configYaml)
        configYaml = local_resolve(configYaml, here);
        if exist(configYaml, 'file')
            cfg = read_module_yaml(configYaml);

            % mtf = module_type field name (valid MATLAB identifier)
            mtf = matlab.lang.makeValidName(m.module_type);

            % check if the config has a field for this module_type and it's a struct
            if isfield(cfg, mtf) && isstruct(cfg.(mtf))
                % inject the parameters from the config into the params struct
                params = cfg.(mtf);
            end
        end
    end

    % --- build edges & node poses ---
    bodies = local_aslist(local_field(m, 'bodies', {}));
    frames = local_aslist(local_field(m, 'frames', {}));
    fts    = local_aslist(local_field(m, 'fixed_transforms', {}));
    jts    = local_aslist(local_field(m, 'joints', {}));

    % --- sanity check ---
    assert(~isempty(bodies), 'Module %s declares no bodies.', m.module_type);

    % initialize an empty array of edges, where each edge is a struct with fields: from, to, T (transformation matrix), isJoint (boolean), and pending (boolean)
    edges = struct('from', {}, 'to', {}, 'T', {}, 'isJoint', {}, 'pending', {});

    % -- build edges from fixed transforms ---
    for k = 1:numel(fts)
        % t = fixed transform struct with fields: from_frame, to_frame, translation, rotation
        t = fts{k};

        % evaluate translation and rotation expressions into numeric values
        tr = local_eval_vec(t.translation, params);
        [R, pend] = local_rot(t.rotation, params);

        % add an edge from the 'from_frame' to the 'to_frame' with the computed transformation matrix T, and mark if it's pending
        edges(end+1) = struct('from', t.from_frame, 'to', t.to_frame, ...
            'T', local_T(R, tr), 'isJoint', false, 'pending', pend); %#ok<AGROW>
    end

    % -- build edges from joints ---
    for k = 1:numel(jts)
        j = jts{k};

        % ax = joint axis vector, qv = joint variable value (e.g., angle for revolute)
        ax = local_eval_vec(j.axis, params);
        qv = local_field(params, j.variable, 0);

        % add an edge from the 'from_frame' to the 'to_frame' with the transformation matrix computed from the joint axis and variable, and mark it as a joint (not pending)
        edges(end+1) = struct('from', j.from_frame, 'to', j.to_frame, ...
            'T', local_T(local_axang(ax, qv), [0;0;0]), ...
            'isJoint', true, 'pending', false); %#ok<AGROW>
    end

    % -- pose computing using forward kinematics --

    % initialize a map to store the global poses of each body/frame, keyed by their names. containers.Map is used for efficient lookup and insertion.
    poses = containers.Map('KeyType', 'char', 'ValueType', 'any');

    % root = first body's center frame at world origin
    rootBody = bodies{1}.name;

    % set the pose of the root body to the identity matrix (4x4), representing its position and orientation at the world origin
    poses(rootBody) = eye(4);

    % Iteratively propagate poses through the edges until no new poses can be computed. This is done by checking if the 'from' frame of an edge has a known pose and the 'to' frame does not. 
    % If so, compute the 'to' frame's pose by multiplying the 'from' frame's pose with the edge's transformation matrix T. Repeat this process until no more changes occur.
    changed = true;

    while changed
        changed = false;

        for k = 1:numel(edges)
            e = edges(k);

            % if the 'from' frame has a known pose and the 'to' frame does not, compute the 'to' frame's pose
            if isKey(poses, e.from) && ~isKey(poses, e.to)
                % forward kinematics
                poses(e.to) = poses(e.from) * e.T;
                changed = true;
            end
        end
    end

    moduleDir = fileparts(moduleYaml);
    repoRoot = fileparts(fileparts(here));

    % --- characteristic scale ---
    maxr = 1;

    % get the name of each pose
    ks = keys(poses);

    for k = 1:numel(ks)
        % get the pose matrix for the current key
        P = poses(ks{k});

        % compute the maximum distance from the origin to all frames
        maxr = max(maxr, norm(P(1:3, 4)));
    end

    % adjust the maximum distance based on the cube length parameter, if provided
    if isfield(params, 'cubeLength'); maxr = max(maxr, params.cubeLength/2); end
    L = max(4, 0.30 * maxr);          % triad axis length

    % --- figure ---
    fig = figure('Name', sprintf('Module: %s', m.module_type), 'Color', 'w');
    ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    view(ax, 135, 25); xlabel(ax, 'X (mm)'); ylabel(ax, 'Y (mm)'); zlabel(ax, 'Z (mm)');

    % world frame
    local_triad(ax, eye(4), L*1.3, 2.5, '-');
    text(ax, 0, 0, 0, '  world', 'FontWeight', 'bold', 'Color', [.2 .2 .2]);

    % bodies as imported geometry when available
    for k = 1:numel(bodies)
        b = bodies{k};
        if ~isKey(poses, b.name); continue; end
        Tb = poses(b.name);
        geomSpec = local_field(b, 'geometry', '');

        geomPath = local_resolve_geometry_path(geomSpec, moduleDir, repoRoot);
        if isempty(geomSpec)
            % No body geometry: keep triads/labels only so pose checks still work.
        elseif isempty(geomPath)
            warning('visualize_module:geometryMissing', ...
                'Body "%s" geometry file not found: %s', b.name, b.geometry);
        else
            geom = local_import_geometry(geomPath);
            if ~isempty(geom)
                local_patch_geometry(ax, Tb, geom, [0.30 0.55 0.85], 0.12);
            end
        end
        local_triad(ax, Tb, L, 1.5, '-');
        text(ax, Tb(1,4), Tb(2,4), Tb(3,4), ...
            ['  ' b.name], 'FontAngle', 'italic', 'Color', [0.1 0.2 0.5]);
    end

    % port / frame triads
    result.module_type = m.module_type;
    result.frames = struct();
    fprintf('\n=== %s (%d DOF, %s) ===\n', m.module_type, ...
        local_field(m, 'dof', 0), local_field(m, 'extraction_status', 'n/a'));

    % get the names of all bodies for later checking of unreached bodies, where @(b) b.name is an anonymous function that extracts the 'name' field from each body struct, and 'UniformOutput', false allows the output to be a cell array of names
    bodyNames = cellfun(@(b) b.name, bodies, 'UniformOutput', false);

    for k = 1:numel(frames)
        f = frames{k};

        if ~isKey(poses, f.name)
            fprintf('  [UNPLACED] %-16s (not reachable from root)\n', f.name);
            continue;
        end

        T = poses(f.name);
        isPort = isfield(f, 'exposed') && isequal(f.exposed, true);
        
        pend = local_frame_pending(edges, f.name);

        if pend
            color = [1 0 1]; lw = 2.0; mk = 'magenta';
        elseif isPort
            color = [0 0 0]; lw = 2.0; mk = 'PORT';
        else
            color = [0.4 0.4 0.4]; lw = 1.0; mk = 'frame';
        end

        local_triad(ax, T, L, lw, local_tern(isPort, '-', '--'));
        plot3(ax, T(1,4), T(2,4), T(3,4), 'o', 'MarkerSize', 5, ...
            'MarkerFaceColor', color, 'MarkerEdgeColor', color);
        lbl = f.name; if pend; lbl = [lbl ' (pending R)']; end
        text(ax, T(1,4), T(2,4), T(3,4), ['  ' lbl], 'Color', color, ...
            'FontWeight', local_tern(isPort, 'bold', 'normal'));
        fn = matlab.lang.makeValidName(f.name);
        result.frames.(fn) = T;
        fprintf('  %-7s %-16s pos=[% 7.2f % 7.2f % 7.2f]  +Z=[% .2f % .2f % .2f]%s\n', ...
            mk, f.name, T(1,4), T(2,4), T(3,4), T(1,3), T(2,3), T(3,3), ...
            local_tern(pend, '  <pending>', ''));
    end

    % unreached bodies note
    for k = 1:numel(bodyNames)
        if ~isKey(poses, bodyNames{k})
            fprintf('  [UNPLACED BODY] %s\n', bodyNames{k});
        end
    end

    title(ax, sprintf('%s  —  %d DOF, %s  (X=red Y=green Z=blue; magenta=pending)', ...
        m.module_type, local_field(m, 'dof', 0), ...
        local_field(m, 'extraction_status', 'n/a')), 'Interpreter', 'none');
    rotate3d(ax, 'on');
end

% ========================================================================
% helpers
% ========================================================================

%% resolve a relative path against the current script's directory
function p = local_resolve(p, here)
    if exist(p, 'file'); return; end

    % combine the current script's directory with the relative path
    cand = fullfile(here, p);

    if exist(cand, 'file'); p = cand; end
end

%% get a field from a struct, or return a default value if the field is missing or empty
function v = local_field(s, name, default)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end

%% make sure the input is a cell array; if it's empty, return {}; if it's already a cell, return it; otherwise, wrap it in a cell
function c = local_aslist(x)
    if isempty(x); c = {}; elseif iscell(x); c = x; else; c = {x}; end
end

%% construct a homogeneous transformation matrix from rotation and translation
function T = local_T(R, t)
    T = eye(4); T(1:3,1:3) = R; T(1:3,4) = t(:);
end

function s = local_tern(cond, a, b)
    if cond; s = a; else; s = b; end
end

%% resolve a body geometry path independent of the current working directory
function p = local_resolve_geometry_path(geomPath, moduleDir, repoRoot)
    p = '';
    if isempty(geomPath)
        return;
    end

    candidates = {};
    if local_is_absolute_path(geomPath)
        candidates{end+1} = geomPath; %#ok<AGROW>
    else
        if startsWith(strrep(geomPath, '\', '/'), 'assets/')
            candidates{end+1} = fullfile(repoRoot, geomPath); %#ok<AGROW>
        end
        candidates{end+1} = fullfile(moduleDir, geomPath); %#ok<AGROW>
        candidates{end+1} = fullfile(repoRoot, geomPath); %#ok<AGROW>
    end

    for k = 1:numel(candidates)
        cand = local_find_case_insensitive_file(candidates{k});
        if ~isempty(cand)
            p = cand;
            return;
        end
    end
end

%% detect whether a path string is absolute on the current platform
function tf = local_is_absolute_path(p)
    if isempty(p)
        tf = false;
    elseif ispc
        tf = ~isempty(regexp(p, '^[A-Za-z]:[\\/]', 'once')) || startsWith(p, '\\');
    else
        tf = startsWith(p, filesep);
    end
end

%% find a file, tolerating case-only filename mismatches such as .STEP vs .step
function p = local_find_case_insensitive_file(pathStr)
    p = '';
    if exist(pathStr, 'file')
        p = pathStr;
        return;
    end

    parentDir = fileparts(pathStr);
    if isempty(parentDir) || ~exist(parentDir, 'dir')
        return;
    end

    [~, name, ext] = fileparts(pathStr);
    targetName = [name ext];
    entries = dir(parentDir);

    for k = 1:numel(entries)
        if ~entries(k).isdir && strcmpi(entries(k).name, targetName)
            p = fullfile(parentDir, entries(k).name);
            return;
        end
    end
end

%% check if a frame is pending (not yet frozen) based on the edges
function pend = local_frame_pending(edges, name)
    pend = false;
    for k = 1:numel(edges)
        % check if the edge's 'to' frame matches the given name and if the edge is marked as pending
        if strcmp(edges(k).to, name) && edges(k).pending
            pend = true; return;
        end
    end
end

%% evaluate a vector expression (cell array of strings or numbers) into a numeric 3x1 vector
function v = local_eval_vec(arr, params)
    % if the input is not a cell array, convert it to a cell array
    if ~iscell(arr); arr = num2cell(arr); end

    % initialize the output vector to zeros
    v = zeros(3,1);

    for k = 1:min(3, numel(arr))
        % evaluate each element of the input array using local_eval_scalar and store it in the output vector
        v(k) = local_eval_scalar(arr{k}, params);
    end
end

%% evaluate a scalar expression (string or number) into a numeric value, using the provided parameters for substitution
function x = local_eval_scalar(e, params)
    if isnumeric(e); x = double(e); return; end
    
    s = e;
    fn = fieldnames(params);

    for i = 1:numel(fn)
        % replace occurrences of the parameter name in the expression with its numeric value, formatted to 12 significant digits
        s = regexprep(s, ['\<' fn{i} '\>'], num2str(params.(fn{i}), '%.12g'));
    end

    if isempty(regexp(s, '^[\s\d\.\+\-\*\/\(\)eE]*$', 'once'))
        error('visualize_module:unresolved', ...
            'Cannot evaluate "%s" — unresolved symbol; add it to the config.', e);
    end

    % convert the evaluated string to a numeric value
    x = str2double(s);

    % if the conversion results in NaN, evaluate the expression using eval (validated arithmetic only)
    if isnan(x); x = eval(s); end %#ok<EVLDM> validated arithmetic only
end

% --- rotation builders ---
%% evaluate a rotation struct into a numeric 3x3 rotation matrix, and indicate if it's pending (not yet frozen)
function [R, pending] = local_rot(rot, params)
    % pending indicates whether a rigid transform or joint rotation
    pending = false;

    % if the input is not a struct, return the identity matrix
    if ~isstruct(rot); R = eye(3); return; end

    % check the type of rotation specified in the struct and compute the corresponding rotation matrix    
    if isfield(rot, 'align')
        % evaluate an alignment struct into a numeric 3x3 rotation matrix
        R = local_align(rot.align);
    elseif isfield(rot, 'pending')
        % if the rotation is marked as pending, return the identity matrix and set pending to true
        R = eye(3); pending = true;
    elseif isfield(rot, 'rpy')
        r = rot.rpy;
        rx = local_eval_scalar(r{1}, params);
        ry = local_eval_scalar(r{2}, params);
        rz = local_eval_scalar(r{3}, params);
        R = local_rotz(rz) * local_roty(ry) * local_rotx(rx);
    elseif isfield(rot, 'axis_angle')
        om = local_eval_vec(rot.axis_angle.omega, params);
        q  = local_eval_scalar(rot.axis_angle.q, params);
        R = local_axang(om, q);
    else
        R = eye(3);
    end
end

%% evaluate an alignment struct into a numeric 3x3 rotation matrix
function R = local_align(al)
    % s = source axis from child frame, d = destination axis in parent frame
    s0 = local_axis(al.a{1}); d0 = local_axis(al.a{2});
    s1 = local_axis(al.b{1}); d1 = local_axis(al.b{2});

    % compute the rotation matrix that aligns the source axes to the destination axes using the cross product to find the third axis
    SRC = [s0 s1 cross(s0, s1)];
    DST = [d0 d1 cross(d0, d1)];

    % SRC * R = DST => R = DST * inv(SRC) = DST * SRC.'
    R = DST * SRC.';   % SRC orthonormal => inv = transpose
end

%% convert a string like 'X', '-Y', 'Z' into a 3x1 axis vector
function v = local_axis(tok)
    % determine the sign of the axis based on the first character of the token
    s = 1; if tok(1) == '-'; s = -1; end

    % initialize the output vector to zeros
    v = zeros(3,1);

    % determine which axis the token corresponds to and set the appropriate component of the vector
    switch upper(tok(end))
        case 'X'; v(1) = s;
        case 'Y'; v(2) = s;
        case 'Z'; v(3) = s;
    end
end

%% convert an axis-angle representation into a rotation matrix using Rodrigues' formula
function R = local_axang(ax, q)
% Rodrigues' rotation formula: R = I + sin(q)*K + (1-cos(q))*K^2, where K is the skew-symmetric matrix of the normalized axis vector

    % n = norm of the axis vector; if it's too small, return the identity matrix
    n = norm(ax); if n < eps; R = eye(3); return; end

    % normalize the axis vector and compute the skew-symmetric matrix K for Rodrigues' rotation formula
    w = ax(:) / n;

    % compute the skew-symmetric matrix K based on the normalized axis vector
    K = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

    % compute the rotation matrix using Rodrigues' formula
    R = eye(3) + sin(q)*K + (1 - cos(q))*(K*K);
end

%% convert a rotation around the X axis into a rotation matrix
function R = local_rotx(a); R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]; end
%% convert a rotation around the Y axis into a rotation matrix
function R = local_roty(a); R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]; end
%% convert a rotation around the Z axis into a rotation matrix
function R = local_rotz(a); R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; end

%% import geometry into surface patch data, preferring the referenced file
function geom = local_import_geometry(geomPath)
    geom = [];

    [~, ~, ext] = fileparts(geomPath);

    if strcmpi(ext, '.stl')
        geom = local_import_stl_geometry(geomPath);
        return;
    end

    warning('visualize_module:geometryUnsupported', ...
        'Unsupported geometry format for visualization: %s', geomPath);
end

function geom = local_import_stl_geometry(stlPath)
    geom = [];

    if exist('stlread', 'file') ~= 2
        warning('visualize_module:stlImportUnavailable', ...
            'Skipping STL geometry %s because stlread is unavailable in this MATLAB environment.', stlPath);
        return;
    end

    try
        mesh = stlread(stlPath);

        if isa(mesh, 'triangulation')
            faces = mesh.ConnectivityList;
            vertices = mesh.Points;
        elseif isstruct(mesh) && isfield(mesh, 'ConnectivityList') && isfield(mesh, 'Points')
            faces = mesh.ConnectivityList;
            vertices = mesh.Points;
        elseif isstruct(mesh) && isfield(mesh, 'faces') && isfield(mesh, 'vertices')
            faces = mesh.faces;
            vertices = mesh.vertices;
        else
            error('Unsupported stlread output type: %s', class(mesh));
        end

        geom = struct('Vertices', vertices, 'Faces', faces);
    catch ME
        warning('visualize_module:stlImportFailed', ...
            'Failed to import STL geometry %s: %s', stlPath, ME.message);
    end
end

% --- drawing ---
%% draw a local triad (coordinate frame) at the origin of the given transformation matrix T, with specified axis length L, line width lw, and line style ls
function local_triad(ax, T, L, lw, ls)
    % o = origin of the triad in 3D space, extracted from the translation component of the transformation matrix T
    o = T(1:3,4);

    % define the colors for the X, Y, and Z axes of the triad (red, green, blue)
    cols = {[0.85 0 0], [0 0.65 0], [0 0 0.9]};

    % loop through each axis (X, Y, Z) and plot a line representing the axis from the origin to the endpoint defined by the axis direction scaled by the length L
    for a = 1:3
        d = T(1:3, a);

        % compute the endpoint
        p1 = o + L * d;

        % plot a line from the origin to the endpoint of the axis in 3D space, using the specified color, line width, and line style
        plot3(ax, [o(1) p1(1)], [o(2) p1(2)], [o(3) p1(3)], ...
            'Color', cols{a}, 'LineWidth', lw, 'LineStyle', ls);

        % draw arrowhead (cone-like basic shape using line segments)
        headL = L * 0.15;  % arrow head length
        headW = L * 0.05;  % arrow head width
        if headL > 0
            % find two orthogonal vectors to form the base of the arrow head
            if abs(d(3)) < 0.9
                u = cross([0; 0; 1], d);
            else
                u = cross([1; 0; 0], d);
            end
            u = u / norm(u);
            v = cross(d, u);

            % base points of the arrow head
            pb = p1 - headL * d;
            p1b1 = pb + headW * u;
            p1b2 = pb - headW * u;
            p1b3 = pb + headW * v;
            p1b4 = pb - headW * v;

            % plot arrow head lines
            hX = [p1(1) p1b1(1) NaN p1(1) p1b2(1) NaN p1(1) p1b3(1) NaN p1(1) p1b4(1)];
            hY = [p1(2) p1b1(2) NaN p1(2) p1b2(2) NaN p1(2) p1b3(2) NaN p1(2) p1b4(2)];
            hZ = [p1(3) p1b1(3) NaN p1(3) p1b2(3) NaN p1(3) p1b3(3) NaN p1(3) p1b4(3)];
            
            line(ax, hX, hY, hZ, 'Color', cols{a}, 'LineWidth', lw, 'LineStyle', '-');
        end
    end
end

%% draw imported geometry at the origin of the given transformation matrix T
function local_patch_geometry(ax, T, geom, color, alpha)
    Vw = (T(1:3,1:3) * geom.Vertices.' + T(1:3,4)).';
    patch(ax, 'Vertices', Vw, 'Faces', geom.Faces, 'FaceColor', color, ...
        'FaceAlpha', alpha, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.35, 'DiffuseStrength', 0.75, 'SpecularStrength', 0.05);
    local_draw_feature_edges(ax, Vw, geom.Faces, [0.75 0.75 0.75], 0.75);
end

%% draw only sharp feature edges so STL triangle mesh lines stay hidden
function local_draw_feature_edges(ax, vertices, faces, color, lineWidth)
    if size(faces, 2) ~= 3
        return;
    end

    try
        tr = triangulation(faces, vertices);
        edgePairs = featureEdges(tr, deg2rad(20));  % 20 degrees threshold for sharp edges
    catch
        edgePairs = [];
    end

    if isempty(edgePairs)
        return;
    end

    X = [vertices(edgePairs(:,1),1) vertices(edgePairs(:,2),1) nan(size(edgePairs,1),1)].';
    Y = [vertices(edgePairs(:,1),2) vertices(edgePairs(:,2),2) nan(size(edgePairs,1),1)].';
    Z = [vertices(edgePairs(:,1),3) vertices(edgePairs(:,2),3) nan(size(edgePairs,1),1)].';
    line(ax, X(:), Y(:), Z(:), 'Color', color, 'LineWidth', lineWidth);
end
