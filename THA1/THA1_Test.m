clear; clc;

Rs = cell(1, 8);

% Test cases for rotm2(?)
Rs{1} = [1 0 0; 0 cos(pi/3) -sin(pi/3); 0 sin(pi/3) cos(pi/3)];
Rs{2} = [cos(pi/3) 0 -sin(pi/3); 0 1 0; sin(pi/3) 0 cos(pi/3)];
Rs{3} = Rs{1}*Rs{2};
% Singular cases for ZYZ
Rs{4} = rotm(so3([pi/6 0 pi/2], "eul", "ZYZ"));
Rs{5} = rotm(so3([pi/12 pi pi/7], "eul", "ZYZ"));
% Singular cases for axis-angle
Rs{6} = eye(3);
% Singular cases for rpy
Rs{7} = rotm(so3([pi/6 pi/2 pi/2], "eul", "ZYX"));
Rs{8} = rotm(so3([pi/12 -pi/2 pi/7], "eul", "ZYX"));

% Test cases for axang2rotm
axangs = {
    [1 0 0 1], ...
    [0 1 0 pi/3], ...
    [3/5 4/5 0 pi/2]
};

% Test cases for quat2rotm
quats = {
    [1/sqrt(2) 1/sqrt(2) 0 0], ...
    [3/5 0 4/5 0], ...
    [1 0 0 0],
};
 

%% Test: m_rotm2axang
% Test singularity
for k=1:length(Rs)
    [ax, ang] = m_rotm2axang(Rs{k});
    assert(m_isequal(Rs{k}, rotm(so3([ax' ang], "axang"))))
end

%% Test: m_rotm2quat
for k=1:length(Rs)
    Q = m_rotm2quat(Rs{k});
    assert(m_isequal(Rs{k}, rotm(so3(Q', "quat"))))
end

%% Test: m_rotm2zyz
for k=1:length(Rs)
    ang = m_rotm2zyz(Rs{k});
    assert(m_isequal(Rs{k}, rotm(so3(ang', "eul", "ZYZ"))))
end

%% Test: m_rotm2rpy
for k=1:length(Rs)
    ang = m_rotm2rpy(Rs{k});
    assert(m_isequal(Rs{k}, rotm(so3(ang', "eul", "ZYX"))))
end

%% Test: m_axang2rotm
for k=1:length(axangs)
    R = m_axang2rotm(axangs{k}(1:3), axangs{k}(4));
    rotm = axang2rotm(axangs{k});
    assert(m_isequal(R, rotm), sprintf("Fail at test case %d", k))
end

%% Test: m_quat2rotm
for k=1:length(quats)
    R = m_quat2rotm(quats{k});
    rotm = quat2rotm(quats{k});
    assert(m_isequal(R, rotm), sprintf("Fail at test case %d", k))
end