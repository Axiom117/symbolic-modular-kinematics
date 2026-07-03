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
