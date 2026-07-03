%% evaluate a vector expression (cell array of strings or numbers) into a numeric 3x1 vector
function v = local_eval_vec(arr, params)
    % if the input is not a cell array, convert it to a cell array
    if ~iscell(arr); arr = num2cell(arr); end

    % initialize the output vector to zeros
    v = zeros(3,1);

    for k = 1:min(3, numel(arr))
        % evaluate each element of the input array using local_eval_scalar and store it in the output vector
        v(k) = local_eval_scalar(arr{k}, params);
    end
end
