%% Testing configurations
q = [0; 2; 0]; % A point on the screw axis
s = [0; 0; 1]; % Scew axis direction
h = 2; % Pitch
t = pi; % Amount of rotation
T = [1 0 0 2; 0 1 0 0; 0 0 1 0; 0 0 0 1]; % Initial configuration

%% Travel along a specified screw
S = m_axang2rotm(s,t/4);
g = [S (eye(3)-S)*q+h*t*s; 0 0 0 1];
frames = cell(5, 1);
frames{1} = T;
for ii = 1:4
    plotFrame(frames{ii})
    frames{ii+1} = g*frames{ii};
end
plotFrame(frames{end})
T1 = frames{end};

%% Calculate screw to travel from T1 to the origin
close all;
R1 = T1(1:3, 1:3);
p1 = T1(1:3, 4);
[ax, ang] = m_rotm2axang(R1');
h = -p1'*R1*ax/norm(ax)^2/ang;
q = lsqminnorm((eye(3)-R1'), (-R1'*p1-h*ang*ax));

plotScrew(q, ax)
plotFrame(T1)


%% Helper functions
function plotFrame(T)
% xyz follows rgb pattern
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    X = [p(1)*ones(1, 3); p(1)+R(1,:)];
    Y = [p(2)*ones(1, 3); p(2)+R(2,:)];
    Z = [p(3)*ones(1, 3); p(3)+R(3,:)];
    colororder(["r", "g", "b"])
    plot3(X, Y, Z, "LineWidth", 3)
    hold on
end

function plotScrew(q, s)
    l = 20;
    P = [q-s*l q+s*l];
    plot3(P(1,:), P(2,:), P(3,:), "LineWidth", 3, "Color", "black");
    hold on
    scatter3(q(1), q(2), q(3), 100, "filled", "MarkerFaceColor","y")
    hold on
end