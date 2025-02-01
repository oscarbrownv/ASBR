function ang = m_rotm2zyz(R)
% m_rotm2quat converts a given SO3 rotation matrix to its equivalent 
% Euler ZYZ representation.
    b = atan2(sqrt(R(3,1)^2+R(3,2)^2), R(3,3));
    a = atan2(R(2,3), R(1,3));
    c = atan2(R(3,2), -R(3,1));
    ang = [a; b; c];
end
