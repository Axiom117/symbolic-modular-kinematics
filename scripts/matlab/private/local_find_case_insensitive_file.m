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
