function R = m_axang2rotm(ax, ang)
% m_axang2rotm.m converts axis-angle to an equivalent rotation matrix.
% 
% Inputs:
%   ax: The rotation axis as a 3 by 1 column vector
%   ang: The rotation angle
%
% Outputs:
%   R: an SO3 rotation matrix

w = skew(ax);
R = eye(3)+w*sin(ang)+w^2*(1-cos(ang));

end