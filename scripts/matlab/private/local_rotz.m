%% convert a rotation around the Z axis into a rotation matrix
function R = local_rotz(a); R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; end
