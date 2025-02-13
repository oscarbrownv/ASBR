function Q = m_rotm2quat(R)
% m_rotm2quat converts a given SO3 rotation matrix to its equivalent 
% quaternion representation.
    [ax, ang] = m_rotm2axang(R);
    Q = [cos(ang/2); ax*sin(ang/2)];
end