function ang = m_rotm2rpy(R)
% m_rotm2quat converts a given SO3 rotation matrix to its equivalent 
% Roll Pitch Yaw (Euler ZYX) representation.
    r = atan2(R(2,1), R(1,1));
    p = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
    y = atan2(R(3,2),R(3,3));
    ang = [r; p; y];
end
