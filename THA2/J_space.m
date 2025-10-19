function J = J_space(S, a)
%J_SPACE Spatial manipulator Jacobian from screw axes.
%   The spatial Jacobian maps joint-rate vectors into the instantaneous twist
%   of the end-effector expressed in the space (world) frame.  Each column is
%   obtained by transforming the corresponding screw axis through the
%   exponential coordinates of the joints that precede it.
%
%   This implementation mirrors equation (4.57) from Modern Robotics and is a
%   handy reference when explaining how the adjoint representation composes
%   twists.
%
% Inputs:
%   S: i-th column is the i-th screw axis described in space frame [wi; vi]
%   a: i-th value is the angle corresponding to the i-th screw axis
%
% Outputs:
%   J : Jacobian expressed in spatial coordinate frame

% Check that the dimension of screw axes is correct
assert(size(S, 1) == 6)
% Check that the length of the screw axes and the angles match
assert(size(S, 2) == length(a))
N = length(a);

% Initialization
if isa(a, "sym")
    J = sym("j", [6, N]);
else
    J = zeros(6, N);
end
A = eye(6);

for ii = 1:N
    % The first column is the first screw axis.  Subsequent columns are the
    % previous adjoint transform multiplied by the new screw axis.
    J(:,ii) = A * S(:,ii);
    % Update the cumulative adjoint by multiplying with the exponential of the
    % current joint motion.  This recursively propagates the motion of upstream
    % joints to the remaining links.
    A = A * Ad(exp_twist(S(:,ii), a(ii)));
end

end