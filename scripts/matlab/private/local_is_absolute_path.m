%% detect whether a path string is absolute on the current platform
function tf = local_is_absolute_path(p)
    if isempty(p)
        tf = false;
    elseif ispc
        tf = ~isempty(regexp(p, '^[A-Za-z]:[\\/]', 'once')) || startsWith(p, '\\');
    else
        tf = startsWith(p, filesep);
    end
end
