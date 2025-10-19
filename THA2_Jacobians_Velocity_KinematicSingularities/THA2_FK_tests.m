clear; clc; close all;

%% Forward-kinematics smoke tests
% Each block below recreates a textbook manipulator and immediately calls
% either `FK_space` or `FK_body`.  The goal is not numerical accuracy
% (MATLAB will plot whatever pose the functions return) but to keep a set of
% quick sanity checks handy while iterating on the FK implementations.
%
% For every scenario we:
%   1. Define the screw axes S expressed in either the space or body frame.
%   2. Specify the joint configuration the book uses as a worked example.
%   3. Build the home configuration matrix M.
%   4. Call the relevant FK routine and visually verify the pose.
% Having all of the canonical manipulators in one file makes it easy to
% recall how twists, exponentials, and the product of exponentials formula
% play together.

%% Planar RR robot (space form)
% First check: a two-link planar arm described in the space frame.  Each
% column of Ss encodes the joint's instantaneous screw axis expressed in the
% {s} frame.
Ss = [0 0 1 0 0 0;
     0 0 1 0 -1 0]';  % revolute joints about z with parallel axes
as = [pi/2, pi/4];     % configuration used in lecture slides
M = [eye(3) [2; 0; 0]; zeros(1, 3), 1];  % straight arm in the home pose

T = FK_space(Ss, as, M);
title("RR robot (space form)")

%% Planar RR robot (body form)
% Repeat the same motion but parameterized in the body frame.  The screw
% axes now encode the twists at the end-effector in the zero configuration.
Ss = [0 0 1 0 2 0;
     0 0 1 0 1 0]';
as = [pi/2, pi/4];
M = [eye(3) [2; 0; 0]; zeros(1, 3), 1];

T = FK_body(Ss, as, M);
title("RR robot (body form)")

%% RRRP robot (space form)
% Three revolute joints followed by a prismatic joint.  Expressing the
% twists in the space frame keeps the linear velocity component intuitive:
% it points from the z-axis of each joint toward the moment arm of the
% end-effector.
Ss = [0 0 1 0 0 0;
      0 0 1 0 -2 0;
      0 0 1 0 -3 0;
      0 0 0 0 0 -1]';
as = [pi/2, pi/4, -pi/2, 1];
M = [eye(3) [3.5; 0; 0.5]; zeros(1, 3), 1];

T = FK_space(Ss, as, M);
title("RRRP robot (space form)")

%% RRRP robot (body form)
% The same geometry but with twists transported to the body frame.  The
% linear velocity components now encode the moment arms relative to the
% tool frame, which is a useful reminder when debugging adjoint transforms.
Ss = [0 0 1 0 3.5 0;
      0 0 1 0 1.5 0;
      0 0 1 0 0.5 0;
      0 0 0 0 0 -1]';
as = [pi/2, pi/4, -pi/2, 1];
M = [eye(3) [3.5; 0; 0.5]; zeros(1, 3), 1];

T = FK_body(Ss, as, M);
title("RRRP robot (body form)")
