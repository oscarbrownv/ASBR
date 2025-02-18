function Q = m_rotm2quat(R)
% m_rotm2quat.m converts a given SO3 rotation matrix to its equivalent 
% quaternion representation.
% 
% Inputs:
%   R: an SO3 rotation matrix
%
% Outputs:
%   Q: The quaternion as a 4 by 1 column vector

[ax, ang] = m_rotm2axang(R);
Q = [cos(ang/2); ax*sin(ang/2)];

end