function [ax, ang] = m_rotm2axang(R)
% m_rotm2axang.m converts a given SO3 rotation matrix to its equivalent 
% axis-angle representation.
% 
% Inputs:
%   R: an SO3 rotation matrix
%
% Outputs:
%   ax: The rotation axis as a 3 by 1 column vector
%   ang: The rotation angle

ang = real(acos((trace(R)-1)/2));

if m_isequal(ang, 0)
    % Handle singular case where the rotation angle is zero
    % Choose zeros as the axis to return
    ax = [0; 0; 0];
elseif m_isequal(ang, pi)
    ax = sqrt(([R(1,1); R(2,2); R(3,3)] + 1)/2);
    ax(2) = sign(R(1,2) + 1e-16)*ax(2);
    ax(3) = sign(R(1,3) + 1e-16)*ax(3);
else
    ax = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]/(2*sin(ang));
end

end