clear; clc; close all;

% DH parameters; [a alpha d theta]
% length: m; angle: radian
dhparams = [0    0     .5   0;
            .25  pi/2  0    0;
            .77  0     0    0;
            -.07 pi/2  .78  0;
            0    -pi/2 0    0;
            0    pi/2  .215 0];
homepos = [0; pi/2; 0; 0; pi/2; pi];

robot = rigidBodyTree("DataFormat","column");

link1 = rigidBody("link1");
j1 = rigidBodyJoint('j1','revolute');
setFixedTransform(j1,dhparams(1,:),'mdh');
link1.Joint = j1;
addBody(robot,link1,'base');

for ii = 2:size(dhparams,1)
    link = rigidBody(sprintf("link%d",ii));
    j = rigidBodyJoint(sprintf("j%d",ii),'revolute');
    j.HomePosition = homepos(ii);
    setFixedTransform(j,dhparams(ii,:),'mdh');
    link.Joint = j;
    addBody(robot,link,sprintf("link%d",ii-1));
end

ee = rigidBody("ee");
tform = trvec2tform([0 0 0]);
setFixedTransform(ee.Joint,tform);
addBody(robot,ee,"link6");

showdetails(robot);
show(robot,"Frames","on");

%% Define screw axis
q0 = homeConfiguration(robot);

% Define screw axes
S = zeros(6, length(q0));
for ii = 1:length(q0)
    T = getTransform(robot,q0,sprintf("link%d", ii));
    w = T(1:3,3);
    q = T(1:3,4);
    S(:,ii) = [w; -cross(w,q)];
end
M = getTransform(robot,q0,"ee");

%% Test cases
q1 = ones(6,1);
q2 = [1; 2; 0.5; -1; -2; -0.5];
test(robot,q0,S,M,homepos)
test(robot,q1,S,M,homepos)
test(robot,q2,S,M,homepos)

function test(robot,q,S,M,homepos)
    % FK
    P = FK_space(S,q-homepos,M);
    Q = getTransform(robot,q,"ee");
    assert(m_isequal(P,Q));

    % Jacobian
    rng(1);
    P = J_space(S,q-homepos);
    Q = geometricJacobian(robot,q,"ee");
    for ii = 1:5
        q_d = rand(6,1)*pi;
        g = FK_space(S,q-homepos,M);
        V = P*q_d;
        v1 = [V(1:3); V(4:6)+cross(V(1:3),g(1:3,4))];
        v2 = Q*q_d;
        assert(m_isequal(v1, v2));
    end
end