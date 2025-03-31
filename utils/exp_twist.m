function T = exp_twist(s, ang)
    w = s(1:3);
    v = s(4:6);
    if m_isequal(w, zeros(size(w)))
        T = [eye(3) v*ang; zeros(1, 3) 1];
    else
        exp_w = m_axang2rotm(w, ang);
        T = [exp_w (eye(3) - exp_w)*cross(w, v) + w * w' * v * ang; zeros(1, 3) 1];
    end
end