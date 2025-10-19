% THA1\_PA3.m
% This script provides a visual walk-through of screw motion concepts used
% in advanced spatial rigid-body robotics.  The first section constructs a
% screw motion directly from axis-angle data and repeatedly applies it to an
% initial configuration.  The second section solves the inverse problem of
% determining the screw motion that maps a final pose back to the world
% origin.  The heavy use of visualisation calls is intentional so that the
% geometric meaning of each computation can be inspected frame-by-frame.

clear; clc; close all;

%% Testing configurations
% Specify the screw parameters that describe a helical motion.  The point q
% and direction s define the screw axis, while the scalar pitch h controls
% the amount of translation generated per radian of rotation.  The value t
% represents the total rotation magnitude that will be applied later in the
% script.
q = [0; 2; 0]; % A point on the screw axis
s = [0; 0; 1]; % Screw axis direction
h = 2;         % Pitch
t = pi;        % Amount of rotation
T = [1 0 0 2; 0 1 0 0; 0 0 1 0; 0 0 0 1]; % Initial configuration (SE(3) matrix)

%% Travel along a specified screw
close all

% Build the incremental screw transformation that corresponds to one quarter
% of the total rotation.  Applying this transformation repeatedly generates a
% sequence of frames along the screw path.  Visualising the intermediate
% frames gives intuition for how rigid bodies move when driven by twists.
S = m_axang2rotm(s,t/4);
g = [S (eye(3)-S)*q+h*t/4*s; 0 0 0 1];
frames = cell(5, 1);
frames{1} = T;
for ii = 1:4
    plotTransforms(se3(frames{ii}), FrameSize=3, FrameAxisLabels="on")
    hold on
    frames{ii+1} = g*frames{ii};
end
plotTransforms(se3(frames{end}), FrameSize=3, FrameAxisLabels="on")
T1 = frames{end};

%% Calculate screw to travel from T1 to the origin
close all;
% Decompose the terminal configuration into rotation and translation
% components.  Working backwards from the final pose, we can compute the
% screw parameters that would map the frame back to the world origin.  This
% uses the fact that any rigid-body displacement can be expressed as a screw
% motion and leverages the logarithm map for SE(3).
R1 = T1(1:3, 1:3);
p1 = T1(1:3, 4);
[ax, ang] = m_rotm2axang(R1');
h = -p1'*R1*ax/norm(ax)^2/ang;
ax = sign(h + 1e-16) * ax;
h = sign(h) * h;
q = lsqminnorm((eye(3)-R1'), (-R1'*p1-h*ang*ax));

T0 = [R1' (eye(3)-R1')*q + h*ang*ax; 0 0 0 1]*T1;
plotScrew(q, ax)
plotTransforms(se3(T1), FrameSize=5, FrameAxisLabels="on")
hold on
plotTransforms(se3(T0), FrameSize=5, FrameAxisLabels="on")
legend()
% plotFrame(T1)

%% Helper functions
function plotScrew(q, s)
    % Visual helper that renders the screw axis defined by the point q and
    % direction s.  The axis is drawn as a straight line and the reference
    % point is highlighted so that viewers can easily relate the algebraic
    % parameters back to the geometry shown in the plots.
    l = 5;
    P = [q-s*l q+s];
    plot3(P(1,:), P(2,:), P(3,:), "LineWidth", 1, "Color", "black", "DisplayName", "Screw axis");
    hold on
    scatter3(q(1), q(2), q(3), 100, "filled", "MarkerFaceColor","y", "DisplayName", "Point on screw axis")
    hold on
end