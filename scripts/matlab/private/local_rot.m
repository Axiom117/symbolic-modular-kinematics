%% evaluate a rotation struct into a numeric 3x3 rotation matrix, and indicate if it's pending (not yet frozen)
function [R, pending] = local_rot(rot, params)
    % pending indicates whether a rigid transform or joint rotation
    pending = false;

    % if the input is not a struct, return the identity matrix
    if ~isstruct(rot); R = eye(3); return; end

    % check the type of rotation specified in the struct and compute the corresponding rotation matrix
    if isfield(rot, 'align')
        % evaluate an alignment struct into a numeric 3x3 rotation matrix
        R = local_align(rot.align);
    elseif isfield(rot, 'pending')
        % if the rotation is marked as pending, return the identity matrix and set pending to true
        R = eye(3); pending = true;
    elseif isfield(rot, 'rpy')
        r = rot.rpy;
        rx = local_eval_scalar(r{1}, params);
        ry = local_eval_scalar(r{2}, params);
        rz = local_eval_scalar(r{3}, params);
        R = local_rotz(rz) * local_roty(ry) * local_rotx(rx);
    elseif isfield(rot, 'axis_angle')
        om = local_eval_vec(rot.axis_angle.omega, params);
        q  = local_eval_scalar(rot.axis_angle.q, params);
        R = local_axang(om, q);
    else
        R = eye(3);
    end
end
