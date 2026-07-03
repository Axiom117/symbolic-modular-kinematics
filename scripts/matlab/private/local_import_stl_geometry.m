%% import an STL mesh into Vertices/Faces patch data
function geom = local_import_stl_geometry(stlPath)
    geom = [];

    if exist('stlread', 'file') ~= 2
        warning('viz_common:stlImportUnavailable', ...
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
        warning('viz_common:stlImportFailed', ...
            'Failed to import STL geometry %s: %s', stlPath, ME.message);
    end
end
