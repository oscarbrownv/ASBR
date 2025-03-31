%% Planar RR Robot
clear; clc;
% Joint 1: s1 = (0, 0, 1), q1 = (0, 0, 0)
% Joint 2: s2 = (0, 0, 1), q2 = (1, 0, 0)
% End effector: qee = (2, 0, 0)

% Screw axes
S = [0 0 1 0 0 0;
     0 0 1 0 -1 0]';
% Rotational angles
a = [pi/18, pi/4];
% Initial configuration; a straight line in x-direction
M = [eye(3) [2; 0; 0]; zeros(1, 3), 1];

Tsd = FK_space(S, a, M);
theta = J_inverse_kinematics(S,Tsd,M,[0;pi/6]);

max(abs(a'-theta))

%% SCARA Robot
clear; clc;
l1 = 2;
l2 = 1;
a = [pi/2; pi/3; pi/6; 0.5]; % Rotational angles
S = [0 0 1 0 0 0;
      0 0 1 l1 0 0;
      0 0 1 l1+l2 0 0;
      0 0 0 0 0 1]';
M = [eye(3) [0; l1+l2; 0]; zeros(1,3) 1];
g = FK_space(S, a, M);
theta = J_inverse_kinematics(S,g,M,zeros(size(a)));