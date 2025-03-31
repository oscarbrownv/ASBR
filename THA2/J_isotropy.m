function n = J_isotropy(J)
% J_isotropy.m calculates the singular number of the manipulability
% ellipsoid.
% The isotropy number is defined as the square root of the condition
% number. As the robot approaches singularities, this number goes to
% infinity.
%
% Inputs:
%   J: the Jacobian at the current configuration
% Outputs:
%   n: the isotropy number
%
s = svd(J);
n = s(1)/s(end);
end
