function [ang, ax] = m_rotm2axang(R)
% m_rotm2axang converts a given SO3 rotation matrix to its equivalent 
% axis-angle representation.
    ang = acos((trace(R)-1)/2);
    if sin(ang) == 0
        ax = [0; 0; 0];
    else
        ax = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]/(2*sin(ang));
    end
end