%% build a joint edge transform for a given kind, axis and variable value
%   T = LOCAL_JOINT_TRANSFORM(KIND, AX, VAL) returns the 4x4 local transform
%   produced by a 1-DOF joint at configuration VAL:
%     - revolute (default): pure rotation VAL (rad) about axis AX (Rodrigues).
%     - prismatic:          pure translation VAL (mm) along axis AX.
%   AX need not be unit length; it is normalized internally.
function T = local_joint_transform(kind, ax, val)
    if nargin < 1 || isempty(kind); kind = 'revolute'; end
    switch lower(kind)
        case 'prismatic'
            n = norm(ax);
            if n < eps; d = [0; 0; 0]; else; d = ax(:) / n * val; end
            T = local_T(eye(3), d);
        otherwise  % revolute
            T = local_T(local_axang(ax, val), [0; 0; 0]);
    end
end
