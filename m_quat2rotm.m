function R = m_quat2rotm(Q)
% m_quat2rotm converts unit quaternion to an equivalent rotation matrix.
    assert(norm(Q) == 1, "Must be unit quaternion")
    ang = 2*acos(Q(1));
    if ang ~= 0
        ax = Q(2:4)/sin(ang/2);
    else
        ax = 0;
    end
    R = m_axang2rotm(ax, ang);
end