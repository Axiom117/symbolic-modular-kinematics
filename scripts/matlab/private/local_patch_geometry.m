%% draw imported geometry at the origin of the given transformation matrix T
function local_patch_geometry(ax, T, geom, color, alpha)
    Vw = (T(1:3,1:3) * geom.Vertices.' + T(1:3,4)).';
    patch(ax, 'Vertices', Vw, 'Faces', geom.Faces, 'FaceColor', color, ...
        'FaceAlpha', alpha, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.35, 'DiffuseStrength', 0.75, 'SpecularStrength', 0.05);
    local_draw_feature_edges(ax, Vw, geom.Faces, [0.75 0.75 0.75], 0.75);
end
