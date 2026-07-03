%% resolve a relative path against the current script's directory
function p = local_resolve(p, here)
    if exist(p, 'file'); return; end

    % combine the current script's directory with the relative path
    cand = fullfile(here, p);

    if exist(cand, 'file'); p = cand; end
end
