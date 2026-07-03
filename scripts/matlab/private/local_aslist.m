%% make sure the input is a cell array; if it's empty, return {}; if it's already a cell, return it; otherwise, wrap it in a cell
function c = local_aslist(x)
    if isempty(x); c = {}; elseif iscell(x); c = x; else; c = {x}; end
end
