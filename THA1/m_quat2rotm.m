function R = m_quat2rotm(Q)
% m_quat2rotm.m converts unit quaternion to an equivalent rotation matrix.
% 
% Inputs:
%   Q: The quaternion as a 4 by 1 column vector
%
% Outputs:
%   R: an SO3 rotation matrix

assert(norm(Q) == 1, "Must be unit quaternion")

ang = 2*acos(Q(1));
if ang ~= 0
    ax = Q(2:4)/sin(ang/2);
else
    ax = [0; 0; 0];
end

R = m_axang2rotm(ax, ang);

end