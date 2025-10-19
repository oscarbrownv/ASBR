%% Inverse-kinematics regression tests
% This script recreates the small examples from lecture to compare the
% damped least-squares solver `J_inverse_kinematics` against the transpose
% based variant `J_transpose_kinematics`.  By solving a planar RR arm and a
% SCARA manipulator we cover both purely revolute and mixed revolute/
% prismatic chains.

%% Planar RR robot
clear; clc;
% Joint 1: s1 = (0, 0, 1), q1 = (0, 0, 0)
% Joint 2: s2 = (0, 0, 1), q2 = (1, 0, 0)
% End effector: qee = (2, 0, 0)

% Define the twists in the space frame.  The second joint acquires a moment
% arm because its axis does not intersect the origin.
S = [0 0 1 0 0 0;
     0 0 1 0 -1 0]';
a = [pi/18, pi/4];  % ground-truth configuration we attempt to recover
M = [eye(3) [2; 0; 0]; zeros(1, 3), 1];

Tsd = FK_space(S, a, M);           % target pose used as "sensor" data
ops = struct('max_iter', 500, 'tol', 1e-3);  % convergence criteria
theta1 = J_inverse_kinematics(S, Tsd, M, [0; 0]);
theta2 = J_transpose_kinematics(S, Tsd, M, [0; 0], ops);

disp("Error (J_inverse_kinematics) =")
disp(norm(Tsd - FK_space(S, theta1, M), "fro"))
disp("Error (J_transpose_kinematics) =")
disp(norm(Tsd - FK_space(S, theta2, M), "fro"))

%% SCARA robot
clear; clc;
l1 = 2;
l2 = 1;
a = [pi/2; pi/3; pi/6; 0.5];  % reference joint configuration
S = [0 0 1 0 0 0;
      0 0 1 l1 0 0;
      0 0 1 l1 + l2 0 0;
      0 0 0 0 0 1]';
M = [eye(3) [0; l1 + l2; 0]; zeros(1, 3) 1];
g = FK_space(S, a, M);
ops = struct('max_iter', 500, 'tol', 1e-3);
theta1 = J_inverse_kinematics(S, g, M, zeros(size(a)));
theta2 = J_transpose_kinematics(S, g, M, zeros(size(a)), ops);

disp("Error (J_inverse_kinematics) =")
disp(norm(g - FK_space(S, theta1, M), "fro"))
disp("Error (J_transpose_kinematics) =")
disp(norm(g - FK_space(S, theta2, M), "fro"))