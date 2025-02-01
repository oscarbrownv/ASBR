% Test rotation matrices
R1 = [1 0 0; 0 cos(pi/3) -sin(pi/3); 0 sin(pi/3) cos(pi/3)];
R2 = [cos(pi/3) 0 -sin(pi/3); 0 1 0; sin(pi/3) 0 cos(pi/3)];
R3 = R1*R2*R1;
Rs = {R1, R2, R3};

axangs = {[1 0 0 1], [0 1 0 pi/3], [3/5 4/5 0 pi/2]};
quats = {[1/sqrt(2) 1/sqrt(2) 0 0], [3/5 0 4/5 0]};
 

%% Test: m_rotm2axang
% Test singularity
[ang, ax] = m_rotm2axang(eye(3));
assert(ang == 0 &&  m_isequal(ax, [0; 0; 0]))

for k=1:length(Rs)
    [ang, ax] = m_rotm2axang(Rs{k});
    axang = rotm2axang(Rs{k})';
    assert(ang == axang(4) && m_isequal(ax, axang(1:3)))
end

%% Test: m_rotm2quat
for k=1:length(Rs)
    Q = m_rotm2quat(Rs{k});
    quat = rotm2quat(Rs{k});
    assert(m_isequal(Q, quat'))
end

%% Test: m_rotm2zyz
for k=1:length(Rs)
    ang = m_rotm2zyz(Rs{k});
    [eul, eulalt] = rotm2eul(Rs{k}, "ZYZ");
    assert(m_isequal(ang, eul') || m_isequal(ang, eulalt'))
end
%% Test: m_rotm2rpy
for k=1:length(Rs)
    ang = m_rotm2rpy(Rs{k});
    [eul, eulalt] = rotm2eul(Rs{k}, "ZYX");
    assert(m_isequal(ang, eul') || m_isequal(ang, eulalt'))
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

function res = m_isequal(a, b)
    tol = 1e-3;
    res = norm(a-b, "fro") < tol;
end