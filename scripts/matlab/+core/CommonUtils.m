classdef CommonUtils
    %COMMONUTILS  Parameter expression evaluation, struct helpers, misc utilities.
    %   All methods are static.  Used across all other smk classes and the
    %   top-level visualization scripts.

    methods (Static)

        %% evaluate a scalar expression (string or number) into a numeric value
        %   Substitutes parameter names with their numeric values.
        function x = evalScalar(e, params)
            if isnumeric(e)
                x = double(e);
                return;
            end
            s = e;
            fn = fieldnames(params);
            for i = 1:numel(fn)
                % replace whole-word occurrences of the parameter name with its numeric value, formatted to 12 significant digits
                s = regexprep(s, ['\<' fn{i} '\>'], num2str(params.(fn{i}), '%.12g'));
            end
            if isempty(regexp(s, '^[\s\d\.\+\-\*\/\(\)eE]*$', 'once'))
                error('viz_common:unresolved', ...
                    'Cannot evaluate "%s" — unresolved symbol; add it to the config.', e);
            end
            x = str2double(s);
            if isnan(x)
                x = eval(s); %#ok<EVLDM> validated arithmetic only
            end
        end

        %% evaluate a vector expression (cell array or numeric) into a 3x1 vector
        function v = evalVec(arr, params)
            if ~iscell(arr)
                arr = num2cell(arr);
            end
            v = zeros(3,1);
            for k = 1:min(3, numel(arr))
                v(k) = core.CommonUtils.evalScalar(arr{k}, params);
            end
        end

        %% get a field from a struct, or return a default value
        function v = field(s, name, default)
            if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                v = s.(name);
            else
                v = default;
            end
        end

        %% ensure the input is a cell array
        function c = asList(x)
            if isempty(x)
                c = {};
            elseif iscell(x)
                c = x;
            else
                c = {x};
            end
        end

        %% ternary helper: return a when cond is true, else b
        function s = tern(cond, a, b)
            if cond
                s = a;
            else
                s = b;
            end
        end

    end
end
