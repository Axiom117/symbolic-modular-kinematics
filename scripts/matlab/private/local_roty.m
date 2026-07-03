%% convert a rotation around the Y axis into a rotation matrix
function R = local_roty(a); R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]; end
