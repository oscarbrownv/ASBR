%% Initialization
clear; clc; close all;

KukaQuantec;

%% Show IK convergence trajectory
qa = ones(6, 1)*pi/12;
A = FK_space(S, qa, M);
qb = rand(6, 1)*2*pi;
B = FK_space(S, qb, M);
disp("Moving from A to B");
disp("A =")
disp(A)
disp("B =")
disp(B)
[q1, traj1] = J_inverse_kinematics(S, B, M, qa);
% Note that Kuka Qunatec is a 6 dof robot which has no redundancy.
% The reason for the difference in trajectories between two methods is due
% to the randomness when handling singularities.
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
[n_iso1, n_cond1] = animation(robot, traj1, S);
[n_iso2, n_cond2] = animation(robot, traj2, S);

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