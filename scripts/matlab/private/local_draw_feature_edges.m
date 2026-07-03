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
