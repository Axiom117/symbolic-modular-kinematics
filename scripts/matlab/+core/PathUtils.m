classdef PathUtils
    %PATHUTILS  File path resolution utilities.
    %   All methods are static.  Handles relative-path resolution,
    %   case-insensitive file finding, and geometry path candidate search.

    methods (Static)

        %% convert a relative path to an absolute path, using a reference directory
        function p = resolve(p, here)
            if exist(p, 'file')
                return;
            end
            cand = fullfile(here, p);
            if exist(cand, 'file')
                p = cand;
            end
        end

        %% resolve a body geometry path independent of the current working directory
        function p = resolveGeometryPath(geomPath, moduleDir, repoRoot)
            p = '';
            if isempty(geomPath)
                return;
            end
            candidates = {};
            if core.PathUtils.isAbsolutePath(geomPath)
                candidates{end+1} = geomPath; %#ok<AGROW>
            else
                if startsWith(strrep(geomPath, '\', '/'), 'assets/')
                    candidates{end+1} = fullfile(repoRoot, geomPath); %#ok<AGROW>
                end
                candidates{end+1} = fullfile(moduleDir, geomPath); %#ok<AGROW>
                candidates{end+1} = fullfile(repoRoot, geomPath); %#ok<AGROW>
            end
            for k = 1:numel(candidates)
                cand = core.PathUtils.findCaseInsensitiveFile(candidates{k});
                if ~isempty(cand)
                    p = cand;
                    return;
                end
            end
        end

        %% find a file, tolerating case-only filename mismatches (e.g. .STEP vs .step)
        function p = findCaseInsensitiveFile(pathStr)
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

        %% detect whether a path string is absolute on the current platform
        function tf = isAbsolutePath(p)
            if isempty(p)
                tf = false;
            elseif ispc
                tf = ~isempty(regexp(p, '^[A-Za-z]:[\\/]', 'once')) || startsWith(p, '\\');
            else
                tf = startsWith(p, filesep);
            end
        end

    end
end
