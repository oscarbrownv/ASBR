clear; clc; close all;

%% Planar RR Robot (Space form)
% Joint 1: s1 = (0, 0, 1), q1 = (0, 0, 0)
% Joint 2: s2 = (0, 0, 1), q2 = (1, 0, 0)
% End effector: qee = (2, 0, 0)

% Screw axes
Ss = [0 0 1 0 0 0;
     0 0 1 0 -1 0]';
% Rotational angles
as = [pi/2, pi/4];
% Initial configuration; a straight line in x-direction
M = [eye(3) [2; 0; 0]; zeros(1, 3), 1];

T = FK_space(Ss, as, M);
title("RR Robot (Space form)")

%% Planar RR Robot (Body form)
% Joint 1: s1 = (0, 0, 1), q1 = (0, 0, 0)
% Joint 2: s2 = (0, 0, 1), q2 = (1, 0, 0)
% End effector: qee = (2, 0, 0)

% Screw axes
Ss = [0 0 1 0 2 0;
     0 0 1 0 1 0]';
% Rotational angles
as = [pi/2, pi/4];
% Initial configuration; a straight line in x-direction
M = [eye(3) [2; 0; 0]; zeros(1, 3), 1];

T = FK_body(Ss, as, M);
title("RR Robot (Body form)")

%% RRRP Robot S(Space form)
% Joint 1 (R): s1 = (0, 0, 1), q1 = (0, 0, 0)
% Joint 2 (R): s2 = (0, 0, 1), q2 = (2, 0, 0)
% Joint 2 (R): s3 = (0, 0, 1), q3 = (3, 0, 0)
% Joint 3 (P): s4 = (0, 0, 1)
% End effector: qee = (3.5, 0, 0.5)

% Screw axes
Ss = [0 0 1 0 0 0;
      0 0 1 0 -2 0
      0 0 1 0 -3 0
      0 0 0 0 0 -1]';
% Rotational angles
as = [pi/2, pi/4, -pi/2, 1];
% Initial configuration; a straight line in x-direction
M = [eye(3) [3.5; 0; 0.5]; zeros(1, 3), 1]; 

T = FK_space(Ss, as, M);
title("RRRP Robot (Space form)")

%% RRRP Robot S(Body form)
% Joint 1 (R): s1 = (0, 0, 1), q1 = (0, 0, 0)
% Joint 2 (R): s2 = (0, 0, 1), q2 = (2, 0, 0)
% Joint 2 (R): s3 = (0, 0, 1), q3 = (3, 0, 0)
% Joint 3 (P): s4 = (0, 0, 1)
% End effector: qee = (3.5, 0, 0.5)

% Screw axes
Ss = [0 0 1 0 3.5 0;
      0 0 1 0 1.5 0
      0 0 1 0 0.5 0
      0 0 0 0 0 -1]';
% Rotational angles
as = [pi/2, pi/4, -pi/2, 1];
% Initial configuration; a straight line in x-direction
M = [eye(3) [3.5; 0; 0.5]; zeros(1, 3), 1];

T = FK_body(Ss, as, M);
title("RRRP Robot (Body form)")