%% convert a rotation around the X axis into a rotation matrix
function R = local_rotx(a); R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]; end
