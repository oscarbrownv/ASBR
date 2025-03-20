function J = J_body(S, a, M)
% J_space.m calculates the body manipulator Jacobian.
% The jacobian maps the joint velocities to the end-effector velocities
% described in the body frame.
% Assume g_st(0) = I!
% 
% Inputs:
%   S: i-th column is the i-th screw axis described in space frame [wi; vi]
%   a: i-th value is the angle corresponding to the i-th screw axis
%   M : initial configuration
%
% Outputs:
%   J : Jacobian expressed in spatial coordinate frame

% Check that the dimension of screw axes is correct
assert(size(S, 1) == 6)
% Check that the length of the screw axes and the angles match
assert(size(S, 2) == length(a))
N = length(a);

% Initialization
J = zeros(6, N);
A = Ad_inv(M);

for ii = N:-1:1
    A = A * Ad_inv(exp_twist(S(:,ii), a(ii)));
    J(:,ii) = A * S(:,ii);
end

end