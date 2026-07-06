function data = readYaml(path)
%READ_YAML  Minimal YAML reader for this project's DSL/module/config files.
%   DATA = READ_YAML(PATH) parses the constrained YAML subset used across
%   this project, returning nested struct (mappings) / cell (sequences) /
%   scalar (double|logical|char).
%
%   Supported subset (sufficient for the L1 module library, no external deps):
%     - block mappings (key: value) nested by indentation
%     - block sequences (- item), incl. multi-line "- key: val" mappings
%     - inline flow mappings { k: v, ... } and sequences [ a, b ]  (nestable)
%     - scalars: int/float, true/false, quoted or bare strings
%     - comments: '#' at line start or preceded by whitespace (so tokens
%       like 'foo.slx#system_1' are preserved)
%
    
    % Load the whole yaml file and use regular expression to split into
    % lines by newline symbols
    raw = fileread(path);
    lines = regexp(raw, '\r\n|\r|\n', 'split');

    % ind: indentation depth; txt: plain text with no space before & after
    items = struct('ind', {}, 'txt', {});

    for k = 1:numel(lines)
        % remove the comments start with # by local_strip_comment()
        ln = local_strip_comment(lines{k});
        % ignore the empty line
        if isempty(strtrim(ln)); continue; end
        % use regular expression to match all whitespace at the beginning
        % of the line
        lead = regexp(ln, '^\s*', 'match', 'once');
        % store all useful info into a flat struct 'item', and use numel()
        % to calculate the number of leading whitespace (indentation)

        % note that the items array is growing in size inside this loop
        % without being preallocated
        items(end+1) = struct('ind', numel(lead), 'txt', strtrim(ln)); %#ok<AGROW>
    end
    if isempty(items); data = struct(); return; end

    % parse the stored info recursively into the struct properties
    [data, ~] = local_parse_block(items, 1, items(1).ind);
end

% ------------------------------------------------------------------------
function out = local_strip_comment(ln)
    % inS: # in single quotation; inD: # in double quotation
    out = ln; inS = false; inD = false;
    for k = 1:numel(ln)
        c = ln(k);
        if c == '''' && ~inD
            inS = ~inS;
        elseif c == '"' && ~inS
            inD = ~inD;
        % only remove comments outside single/double quotation
        elseif c == '#' && ~inS && ~inD
            % comment either starts in the beginning of a line, or after a
            % space in the end of line
            if k == 1 || isspace(ln(k-1))
                % trim the comment part from the line, k == 1 will result
                % in an empty character array
                out = ln(1:k-1);
                return;
            end
        end
    end
end

%% This function determines whether parse a 'Sequence' or a 'Mapping' depending on the first character and its indentation
function [val, i] = local_parse_block(items, i, indent)

    % obtain number of lines
    n = numel(items);

    % the first character '-' indicates a YAML list (exp, multi-line
    % content under 'frames:' or 'fixed_transforms:')
    if items(i).txt(1) == '-'
        % --- sequence ---

        % initialize the cell for val
        val = {};

        % the subsequent lines belong to the same list as long as their
        % indentation are the same and start with '-'
        while i <= n && items(i).ind == indent && items(i).txt(1) == '-'
            
            % retrieve the plain text without '-'
            rest = strtrim(items(i).txt(2:end));
            
            % if content after '-' is empty, recursively call
            % local_parse_block
            if isempty(rest)
                i = i + 1;
                [el, i] = local_parse_block(items, i, items(i).ind);
                val{end+1} = el; %#ok<AGROW>
                
            % '{' or '[' after '-' indicates inline flow content, call
            % local_parse flow() to parse
            elseif rest(1) == '{' || rest(1) == '['
                val{end+1} = local_parse_flow(rest); %#ok<AGROW>
                i = i + 1;
            
            % multi-line "- key: val" mapping element
            elseif ~isempty(regexp(rest, '^[^:]+:', 'once'))
                
                % calculate the child indentation
                childIndent = indent + 2;

                % buffer the txt and register the child indentation
                buf = items(i); buf.txt = rest; buf.ind = childIndent;

                i = i + 1;
                
                % bundle the multi-line text that belong to this element into
                % the buffer
                while i <= n && items(i).ind >= childIndent
                    buf(end+1) = items(i); %#ok<AGROW>
                    i = i + 1;
                end

                % recursively parse it into struct stored in 'val{end+1}'
                [el, ~] = local_parse_block(buf, 1, childIndent);
                val{end+1} = el; %#ok<AGROW>

            else
                val{end+1} = local_parse_scalar(rest); %#ok<AGROW>
                i = i + 1;
            end
        end
    
    % beginning of the line is not '-' indicates this layer is a mapping
    else
        % --- mapping ---
        % mapping is expressed as struct in MATLAB
        val = struct();

        % parse all lines with the same indent and no '-' into the fields
        while i <= n && items(i).ind == indent && items(i).txt(1) ~= '-'
            
            % call local_split_kv() to divide key and value by the first
            % colon
            [key, after] = local_split_kv(items(i).txt);

            % after ':' is empty indicates the value is nested in the next
            % layer
            if isempty(after)
                i = i + 1;

                % check if the next line is indented more than the current line
                if i <= n && items(i).ind > indent

                    % recursively call local_parse_block() to parse the next layer
                    [child, i] = local_parse_block(items, i, items(i).ind);

                else
                    child = [];
                end

                % assign the parsed child struct to the current key in the mapping
                val.(key) = child;

            else

                % if after ':' is not empty, parse the value directly
                val.(key) = local_parse_value(after);
                
                % increment the index to move to the next line
                i = i + 1;
            end
        end
    end
end

% ------------------------------------------------------------------------
function v = local_parse_value(s)
    s = strtrim(s);
    if ~isempty(s) && (s(1) == '{' || s(1) == '[')
        v = local_parse_flow(s);
    else
        v = local_parse_scalar(s);
    end
end

% ------------------------------------------------------------------------
%% This function parses a flow mapping or sequence, which can be nested.
function v = local_parse_flow(s)
    
    s = strtrim(s);

    if isempty(s); v = []; return; end

    if s(1) == '{'
        inner = strtrim(s(2:end-1));
        v = struct();

        if isempty(inner); return; end

        % split the inner content by top-level commas, ignoring commas inside nested structures
        parts = local_split_top(inner, ',');
        
        for k = 1:numel(parts)
            [key, after] = local_split_kv(parts{k});
            v.(key) = local_parse_value(after);
        end
    elseif s(1) == '['
        inner = strtrim(s(2:end-1));
        if isempty(inner); v = {}; return; end
        parts = local_split_top(inner, ',');
        v = cell(1, numel(parts));
        for k = 1:numel(parts)
            v{k} = local_parse_value(strtrim(parts{k}));
        end
    else
        v = local_parse_scalar(s);
    end
end

% ------------------------------------------------------------------------
%% This function splits a string by a delimiter, ignoring delimiters inside nested structures.
function parts = local_split_top(s, delim)
    % initialize parts as cell array
    parts = {}; 

    % depth: current nesting depth; inS: inside single quotes; inD: inside double quotes; last: index of the last split
    depth = 0; inS = false; inD = false; last = 1;

    % loop through each character in the string
    for k = 1:numel(s)
        c = s(k);

        % toggle inS and inD flags when encountering quotes, but only if not already inside the other type of quote
        if c == '''' && ~inD
            inS = ~inS;
        elseif c == '"' && ~inS
            inD = ~inD;
        elseif ~inS && ~inD
            if c == '{' || c == '['
                depth = depth + 1;
            elseif c == '}' || c == ']'
                depth = depth - 1;

            % if the current character is the delimiter and we are at the top level (depth == 0), we split the string
            elseif c == delim && depth == 0

                % split the string at the delimiter and add the part to the parts cell array
                parts{end+1} = s(last:k-1); %#ok<AGROW>
                last = k + 1;
            end
        end
    end

    % add the last part
    tail = s(last:end);
    if ~isempty(strtrim(tail)) || ~isempty(parts)

        % add the last part of the string after the final delimiter to the parts cell array
        parts{end+1} = tail; %#ok<AGROW>
    end
end

% ------------------------------------------------------------------------
function [key, after] = local_split_kv(s)
    ci = strfind(s, ':');
    ci = ci(1);
    key = strtrim(s(1:ci-1));
    key = strrep(strrep(key, '"', ''), '''', '');
    after = strtrim(s(ci+1:end));
end

% ------------------------------------------------------------------------
%% This function parses a scalar value, which can be a number, boolean, or string.
function v = local_parse_scalar(s)
    s = strtrim(s);
    if isempty(s); v = []; return; end

    % check if the string is quoted, and remove the quotes if so
    if (s(1) == '"' && s(end) == '"') || (s(1) == '''' && s(end) == '''')
        v = s(2:end-1);
        return;
    end

    % check if the string is a boolean value
    if strcmp(s, 'true');  v = true;  return; end
    if strcmp(s, 'false'); v = false; return; end

    % check if the string is a valid number using regular expression, including integers, decimals, and scientific notation
    if ~isempty(regexp(s, '^[+-]?\d+(\.\d+)?([eE][+-]?\d+)?$', 'once'))
        v = str2double(s);
        return;
    end

    % if none of the above, return the string as is (exp. cubeLength/2)
    v = s;
end
