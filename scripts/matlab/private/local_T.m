%% construct a homogeneous transformation matrix from rotation and translation
function T = local_T(R, t)
    T = eye(4); T(1:3,1:3) = R; T(1:3,4) = t(:);
end
