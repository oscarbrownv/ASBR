function ang = m_rotm2rpy(R)
% m_rotm2quat.m converts a given SO3 rotation matrix to its equivalent 
% Roll Pitch Yaw (Euler ZYX) representation.
% 
% Inputs:
%   R: an SO3 rotation matrix
%
% Outputs:
%   ax: The rpy angles as a 3 by 1 column vector


p = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
if m_isequal(cos(p), 0)
    % Handle the singular case where b is 0
    % Choose r = 0
    y = atan2(-R(1,2)/R(3,1), R(2,2));
    ang = [0; p; y];
else
    r = atan2(R(2,1), R(1,1));
    y = atan2(R(3,2),R(3,3));
    ang = [r; p; y];
end

end
