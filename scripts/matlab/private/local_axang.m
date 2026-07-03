%% convert an axis-angle representation into a rotation matrix using Rodrigues' formula
function R = local_axang(ax, q)
% Rodrigues' rotation formula: R = I + sin(q)*K + (1-cos(q))*K^2, where K is the skew-symmetric matrix of the normalized axis vector

    % n = norm of the axis vector; if it's too small, return the identity matrix
    n = norm(ax); if n < eps; R = eye(3); return; end

    % normalize the axis vector and compute the skew-symmetric matrix K for Rodrigues' rotation formula
    w = ax(:) / n;

    % compute the skew-symmetric matrix K based on the normalized axis vector
    K = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

    % compute the rotation matrix using Rodrigues' formula
    R = eye(3) + sin(q)*K + (1 - cos(q))*(K*K);
end
