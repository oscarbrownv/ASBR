%% Load Kuka Iiwa 7
clear; clc; close all;

iiwa = loadrobot("kukaIiwa7", "DataFormat", "column");
showdetails(iiwa);
q0 = homeConfiguration(iiwa);

% Define screw axes
S = zeros(6, length(q0));
for ii = 1:length(q0)
    T = getTransform(iiwa, q0, sprintf("iiwa_link_%d", ii));
    w = T(1:3,3);
    q = T(1:3,4);
    S(:,ii) = [w; -cross(w,q)];
end
M = getTransform(iiwa, q0, "iiwa_link_ee");

%% Check forward kinematics
% Home configuration
P = FK_space(S, q0, M);
Q = getTransform(iiwa, q0, "iiwa_link_ee");
assert(m_isequal(P, Q));

% Configuration 1
q = ones(7, 1);
P = FK_space(S, q, M);
Q = getTransform(iiwa, q, "iiwa_link_ee");
assert(m_isequal(P, Q));

% Configuration 2
q = [1; 2; 0.5; -1; -2; -0.5; 0.25];
P = FK_space(S, q, M);
Q = getTransform(iiwa, q, "iiwa_link_ee");
assert(m_isequal(P, Q));

%% Check Jacobian
rng(2);
q_d = rand(7, 1)*pi;

% Home configuration
P = J_space(S, q0);
Q = geometricJacobian(iiwa, q0, "iiwa_link_ee");
V = P*q_d;
v1 = [V(1:3); V(4:6) + cross(V(1:3),M(1:3,4))];
v2 = Q*q_d;
assert(m_isequal(v1, v2));

% Configuration 1
q = ones(7, 1);
P = J_space(S, q);
Q = geometricJacobian(iiwa, q, "iiwa_link_ee");
V = P*q_d;
g = FK_space(S, q, M);
v1 = [V(1:3); V(4:6) + cross(V(1:3),g(1:3,4))];
v2 = Q*q_d;
assert(m_isequal(v1, v2));

% Configuration 2
q = [1; 2; 0.5; -1; -2; -0.5; 0.25];
P = J_space(S, q);
Q = geometricJacobian(iiwa, q, "iiwa_link_ee");
V = P*q_d;
g = FK_space(S, q, M);
v1 = [V(1:3); V(4:6) + cross(V(1:3),g(1:3,4))];
v2 = Q*q_d;
assert(m_isequal(v1, v2));

%% Show IK convergence trajectory
qa = ones(7, 1)*pi/12;
A = FK_space(S, qa, M);
qb = rand(7, 1)*2*pi;
B = FK_space(S, qb, M);
disp("Moving from A to B");
disp("A =")
disp(A)
disp("B =")
disp(B)
[q1, traj1] = J_inverse_kinematics(S, B, M, qa);
[q2, traj2] = redundancy_resolution(S, B, M, qa);

disp("FK(q1) = ")
disp(FK_space(S, q1, M));
disp("error (Frobenius norm) =")
disp(norm(B-FK_space(S, q1, M),"fro"))

disp("FK(q2) = ")
disp(FK_space(S, q2, M));
disp("error (Frobenius norm) =")
disp(norm(B-FK_space(S, q2, M),"fro"))

%% Move from configuration A to B
[n_iso1, n_cond1] = animation(iiwa, traj1, S);
[n_iso2, n_cond2] = animation(iiwa, traj2, S);

function [n_iso, n_cond] = animation(robot, traj, S)
    figure
    rate = 10;
    r = rateControl(rate);
    L = 5;
    n_iso = zeros((L-1)*(size(traj,1)-1),1);
    n_cond = zeros((L-1)*(size(traj,1)-1),1);
    for ii = 1:size(traj,1)-1
        c = linspace(0, 1, L);
        for jj = 1:L-1
            x = traj(ii,:)+(traj(ii+1,:)-traj(ii,:))*c(jj);
            J = J_space(S, x');
            subplot(2,3,[1,4]);
            show(robot,x',"PreservePlot",false);
            subplot(2,3,2);
            ellipsoid_plot_angular(J(1:3,:))
            subplot(2,3,5);
            ellipsoid_plot_linear(J(4:6,:))
            subplot(2,3,3)
            n_iso((ii-1)*(L-1)+jj) = J_isotropy(J);
            plot(1:length(n_iso), n_iso)
            subplot(2,3,6)
            n_cond((ii-1)*(L-1)+jj) = J_condition(J);
            plot(1:length(n_cond), n_cond)
            drawnow;
            waitfor(r);
        end
    end
    subplot(2,3,[1,4]);
    show(robot,traj(end,:)');
end



