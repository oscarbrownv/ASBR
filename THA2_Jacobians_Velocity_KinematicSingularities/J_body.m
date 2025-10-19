function J = J_body(S, a, M)
%J_BODY Body-frame manipulator Jacobian from screw axes.
%   The body Jacobian relates joint velocities to the twist of the
%   end-effector expressed in its own instantaneous frame.  It is useful when
%   performing inverse kinematics via body twists or when comparing with
%   operational-space control formulations.
%
%   Similar to the spatial Jacobian, this routine applies adjoint transforms
%   sequentially.  Working backwards through the kinematic chain is more
%   convenient for the body frame because each column depends on joints that
%   come after it.
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
if isa(a, "sym")
    J = sym("j", [6, N]);
else
    J = zeros(6, N);
end
A = Ad_inv(M);

for ii = N:-1:1
    % Propagate the adjoint transform backwards.  Each multiplication removes
    % the effect of one more joint from the tip towards the base.
    A = A * Ad_inv(exp_twist(S(:,ii), a(ii)));
    % The resulting column represents the screw axis expressed in the body
    % frame after accounting for downstream joints.
    J(:,ii) = A * S(:,ii);
end

end