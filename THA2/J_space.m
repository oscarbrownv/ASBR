function J = J_space(S, a)
% J_space.m calculates the spatial manipulator Jacobian.
% The jacobian maps the joint velocities to the end-effector velocities
% described in the space frame.
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
    J(:,ii) = A * S(:,ii);
    A = A * Ad(exp_twist(S(:,ii), a(ii)));
end

end