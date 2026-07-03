%% evaluate a scalar expression (string or number) into a numeric value, using the provided parameters for substitution
function x = local_eval_scalar(e, params)
    if isnumeric(e); x = double(e); return; end

    s = e;
    fn = fieldnames(params);

    for i = 1:numel(fn)
        % replace occurrences of the parameter name in the expression with its numeric value, formatted to 12 significant digits
        s = regexprep(s, ['\<' fn{i} '\>'], num2str(params.(fn{i}), '%.12g'));
    end

    if isempty(regexp(s, '^[\s\d\.\+\-\*\/\(\)eE]*$', 'once'))
        error('viz_common:unresolved', ...
            'Cannot evaluate "%s" — unresolved symbol; add it to the config.', e);
    end

    % convert the evaluated string to a numeric value
    x = str2double(s);

    % if the conversion results in NaN, evaluate the expression using eval (validated arithmetic only)
    if isnan(x); x = eval(s); end %#ok<EVLDM> validated arithmetic only
end
