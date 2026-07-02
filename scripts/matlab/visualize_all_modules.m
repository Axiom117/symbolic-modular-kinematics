function visualize_all_modules(configYaml)
%VISUALIZE_ALL_MODULES  Validate & visualize every module in specs/modules/.
%   VISUALIZE_ALL_MODULES() uses module_viz_config.yaml next to this file.
%   VISUALIZE_ALL_MODULES(CONFIGYAML) uses a custom config path.
%
%   Opens one figure per module and prints a textual frame report for each,
%   so the whole L1 library can be checked in a single call.

    if nargin < 1 || isempty(configYaml)
        configYaml = 'module_viz_config.yaml';
    end
    here = fileparts(mfilename('fullpath'));
    modDir = fullfile(here, '..', '..', 'specs', 'modules');
    files = dir(fullfile(modDir, '*.yaml'));
    assert(~isempty(files), 'No module YAMLs found in %s', modDir);

    for k = 1:numel(files)
        f = fullfile(files(k).folder, files(k).name);
        try
            visualize_module(f, configYaml);
        catch err
            fprintf(2, '\n[FAILED] %s\n  %s\n', files(k).name, err.message);
        end
    end
    fprintf('\nDone: %d module(s) processed.\n', numel(files));
end
