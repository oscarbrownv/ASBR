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

ang = acos((trace(R)-1)/2);

if sin(ang) == 0
    % Handle singular case where the rotation angle is zero
    % Choose zeros as the axis to return
    ax = [1; 0; 0];
else
    ax = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]/(2*sin(ang));
end

end