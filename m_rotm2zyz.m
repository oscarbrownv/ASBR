function ang = m_rotm2zyz(R)
% m_rotm2quat.m converts a given SO3 rotation matrix to its equivalent 
% Euler ZYZ representation.
% 
% Inputs:
%   R: an SO3 rotation matrix
%
% Outputs:
%   ax: The ZYZ Euler angles as a 3 by 1 column vector

b = atan2(sqrt(R(3,1)^2+R(3,2)^2), R(3,3));
if m_isequal(sin(b), 0)
    % Handle the singular case where b is 0
    % Choose a = 0
    c = atan2(R(2,1), R(1,1)/R(3,3));
    ang = [0; b; c];
else
    a = atan2(R(2,3), R(1,3));
    c = atan2(R(3,2), -R(3,1));
    ang = [a; b; c];
end

end
