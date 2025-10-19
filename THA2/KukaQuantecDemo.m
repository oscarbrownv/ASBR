%% Initialization
% Demonstration harness for comparing the three IK solvers on the KUKA
% Quantec arm.  Running `KukaQuantec` populates the shared workspace
% variables (S, M, robot, homepos) used in the follow-up tests.
clear; clc; close all;
KukaQuantec;
rng(70);  % fix randomness so repeated runs yield the same trajectories

%% Show IK convergence trajectory
% Pick two random configurations A and B and evaluate how each solver walks
% between them.  Because the Quantec arm is square (6-DoF robot with a
% 6-D task space), all three algorithms should converge to the same pose up
% to numerical tolerances.
qa = zeros(6, 1);
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
[q3, traj3] = J_transpose_kinematics(S, B, M, qa, struct());

disp("FK(q1) = ")
disp(FK_space(S, q1, M));
disp("J_inverse_kinematics error (Frobenius norm) =")
disp(norm(B - FK_space(S, q1, M), "fro")/norm(B, "fro"))

disp("FK(q2) = ")
disp(FK_space(S, q2, M));
disp("redundancy_resolution error (Frobenius norm) =")
disp(norm(B - FK_space(S, q2, M), "fro")/norm(B, "fro"))

disp("FK(q3) = ")
disp(FK_space(S, q3, M));
disp("J_transpose_kinematics error (Frobenius norm) =")
disp(norm(B - FK_space(S, q3, M), "fro")/norm(B, "fro"))

%% Move from configuration A to B
% [n_iso1, n_cond1] = animation(robot,traj1,S,homepos,"J_inverse_kinematics_70",true);
[n_iso2, n_cond2] = animation(robot, traj2, S, homepos,"redundancy_resolution_70",true);
% [n_iso3, n_cond3] = animation(robot, traj3, S, homepos,"J_transpose_kinematics_100",false);

function [n_iso, n_cond] = animation(robot, traj, S, homepos, filename, record)
% Visualize a joint-space trajectory while plotting manipulability metrics
% and optionally recording the frames to disk.
    figure
    rate = 10;
    r = rateControl(rate);  % throttle the animation loop to a steady fps
    n_iso = zeros(size(traj,1),1);
    n_cond = zeros(size(traj,1),1);  % storage for isotropy/condition metrics
    if record
        v = VideoWriter(sprintf("videos/%s.mp4",filename),"MPEG-4");
        v.FrameRate = rate;
        open(v)
    end
    for ii = 1:size(traj,1)
        x = traj(ii,:);
        J = J_space(S, x');
        subplot(2,3,[1 4]);
        show(robot,x'+homepos,"PreservePlot",false,"FastUpdate",true);  % pose trace
        subplot(2,3,2);
        ellipsoid_plot_angular(J(1:3,:))  % rotational manipulability
        axis tight;
        subplot(2,3,5);
        ellipsoid_plot_linear(J(4:6,:))  % translational manipulability
        axis tight;
        subplot(2,3,3)
        n_iso(ii) = J_isotropy(J);
        plot(1:length(n_iso), n_iso)
        title("Isotropy Number")
        axis tight;
        subplot(2,3,6)
        n_cond(ii) = J_condition(J);
        plot(1:length(n_cond), n_cond)
        title("Condition Number")
        axis tight;
        waitfor(r);  % allow rateControl to enforce the desired frame rate
        if record
            writeVideo(v,getframe(gcf));  % capture the current figure
        end
    end
    if record
        close(v);
    end
end