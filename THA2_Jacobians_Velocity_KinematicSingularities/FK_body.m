function T = FK_body(Ss, as, M)
% FK_space.m calculates the body form forward kinematics (FK).
% Assume ||w|| == 1.
% 
% Inputs:
%   Ss: i-th column is the i-th screw axis [wi; vi]
%   as: i-th value is the angle corresponding to the i-th screw axis
%   M : initial configuration
%
% Outputs:
%   T : the transformation matrix representing the pose of the end effector
%       in the world coordinate frame

arguments
    Ss (6,:) double
    as (1,:) double
    M (4,4) double
end

% Check that the dimension of screw axes is correct
assert(size(Ss, 1) == 6)

% Check that the length of the screw axes and the angles match
assert(size(Ss, 2) == length(as))
N = length(as);

T = M;
for ii = 1:N
    T = T * exp_twist(Ss(:, ii), as(ii));
end

plotTransforms(T(1:3,4)', m_rotm2quat(T(1:3,1:3))')
hold on
end