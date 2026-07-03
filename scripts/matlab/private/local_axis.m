%% convert a string like 'X', '-Y', 'Z' into a 3x1 axis vector
function v = local_axis(tok)
    % determine the sign of the axis based on the first character of the token
    s = 1; if tok(1) == '-'; s = -1; end

    % initialize the output vector to zeros
    v = zeros(3,1);

    % determine which axis the token corresponds to and set the appropriate component of the vector
    switch upper(tok(end))
        case 'X'; v(1) = s;
        case 'Y'; v(2) = s;
        case 'Z'; v(3) = s;
    end
end
