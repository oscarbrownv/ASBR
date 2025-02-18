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

function S = skew(s)
    S = [0 -s(3) s(2); s(3) 0 -s(1); -s(2) s(1) 0];
end