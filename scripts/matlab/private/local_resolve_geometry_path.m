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
