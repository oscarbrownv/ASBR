function cond = singularity(S)
% singularity.m calculates singular configurations of the robot.
% At a singular configuration the Jacobian drops rank.
% Hence, the Jacobian fails to be invertible at singular points and
% becomes unable to move in certain directions.
%
% Inputs:
%   S: i-th column is the i-th screw axis described in space frame [wi; vi]
%
% Outputs:
%   cond: conditions of singularities

n = size(S, 2);
syms x [n 1] real
syms z [n 1] real

J = J_space(S, x);

sols = solve(J * z == 0, z, "ReturnConditions", true);
cond = sols.conditions;
end