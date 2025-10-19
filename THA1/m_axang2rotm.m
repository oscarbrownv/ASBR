function R = m_axang2rotm(ax, ang)
%M_AXANG2ROTM Convert axis-angle data into a rotation matrix.
%
%   R = M_AXANG2ROTM(ax, ang) evaluates Rodrigues' rotation formula to turn
%   the exponential coordinates (ax, ang) into a 3x3 matrix in SO(3).  This is
%   the forward direction of the exponential map and is central to computing
%   rigid-body motions from twists.

w = skew(ax);
R = eye(3)+w*sin(ang)+w^2*(1-cos(ang));

end