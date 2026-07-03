%% ternary helper: return a when cond is true, else b
function s = local_tern(cond, a, b)
    if cond; s = a; else; s = b; end
end
