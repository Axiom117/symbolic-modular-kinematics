%% import geometry into surface patch data, preferring the referenced file
function geom = local_import_geometry(geomPath)
    geom = [];

    [~, ~, ext] = fileparts(geomPath);

    if strcmpi(ext, '.stl')
        geom = local_import_stl_geometry(geomPath);
        return;
    end

    warning('viz_common:geometryUnsupported', ...
        'Unsupported geometry format for visualization: %s', geomPath);
end
