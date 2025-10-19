%% Jacobian sanity checks for the SCARA manipulator
% Recreates the analytical expressions from the notes to make sure the
% Jacobian routines agree with theory.  Keeping this script around is useful
% when refactoring the adjoint helpers, since a mismatch will immediately
% trigger the assertions below.

%% SCARA robot spatial Jacobian
clc; clear; close all;
l1 = 1;
l2 = 1;
a = [pi/2; pi/3; pi/6; 0.5];  % joint values pulled from lecture example
S = [0 0 1 0 0 0;
      0 0 1 l1 0 0;
      0 0 1 l1 + l2 0 0;
      0 0 0 0 0 1]';
Js = J_space(S, a);
J_ref = [0 0 0 0;
         0 0 0 0;
         1 1 1 0;
         0 l1*cos(a(1)) l1*cos(a(1)) + l2*cos(a(1) + a(2)) 0;
         0 l1*sin(a(1)) l1*sin(a(1)) + l2*sin(a(1) + a(2)) 0;
         0 0 0 1;];

% Check if the results are correct
assert(m_isequal(Js, J_ref))

%% SCARA robot body Jacobian
% If you need an analytical reference for Jb, derive the closed-form using
% the adjoint relationship below.  Leaving this note as a breadcrumb saved me
% from re-deriving it during the course.

%% Check the relationship between space and body Jacobians
M = [eye(3) [0; l1 + l2; 0]; zeros(1, 3) 1];
Jb = J_body(S, a, M);
g = FK_space(S, a, M);
assert(m_isequal(Js, Ad(g) * Jb))

%% Check ellipsoid plots
Jw = Jb(1:3, :);
Jv = Jb(4:6, :);
ellipsoid_plot_angular(Jw);
ellipsoid_plot_linear(Jv);


