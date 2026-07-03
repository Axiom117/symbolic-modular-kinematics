%% evaluate an alignment struct into a numeric 3x3 rotation matrix
function R = local_align(al)
    % s = source axis from child frame, d = destination axis in parent frame
    s0 = local_axis(al.a{1}); d0 = local_axis(al.a{2});
    s1 = local_axis(al.b{1}); d1 = local_axis(al.b{2});

    % compute the rotation matrix that aligns the source axes to the destination axes using the cross product to find the third axis
    SRC = [s0 s1 cross(s0, s1)];
    DST = [d0 d1 cross(d0, d1)];

    % SRC * R = DST => R = DST * inv(SRC) = DST * SRC.'
    R = DST * SRC.';   % SRC orthonormal => inv = transpose
end
