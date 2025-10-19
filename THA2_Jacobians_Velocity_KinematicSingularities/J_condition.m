function n = J_condition(J)
% J_conditon.m calculates the condition number of the manipulability
% ellipsoid.
% The condition number is defined as the ratio between the largest and the
% smallest singular values of J*J'
%
% Inputs:
%   J: the Jacobian at the current configuration
% Outputs:
%   n: the condition number
%
n = J_isotropy(J)^2;
end
