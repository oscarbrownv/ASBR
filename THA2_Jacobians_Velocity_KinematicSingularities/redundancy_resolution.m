function [theta, traj] = redundancy_resolution(S, Tsd, M, theta0)
% J_inverse_kinematics.m calculates the joint configuration given the
% desired end effector position and orientation.
% 
% Inputs:
%   S: i-th column is the i-th screw axis [wi; vi]
%   Tsd: desired end effector pose expressed in the space frame
%   M : initial configuration
%   theta0: initial guess
%
% Outputs:
%   theta: joint configuration

tol = 1e-3;
iter_max = 1000;
n = length(theta0);
k0 = 40;

theta = theta0;
Tbd = FK_space(S, theta, M)\Tsd;
s = m_logm(Tbd);

cnt = 1;
traj = zeros(iter_max, n);
traj(1, :) = theta0;
while norm(s) > tol && cnt < iter_max
    J = J_body(S, theta, M);
    if rank(J*J') < min(6,size(S,2))
        theta = theta + rand(n,1)*0.01;
        disp("Encounter singularity @")
        disp(theta)
    else
        a = pinv(J)*s;
        b = (eye(n)-pinv(J)*J)*grad_w(S, theta, M)*k0;
        % TODO: disable mod if the robot has p joint
        theta = mod(theta + a + b, 2*pi);
    end
    Tbd = FK_space(S, theta, M)\Tsd;
    s = m_logm(Tbd);
    cnt = cnt+1;
    traj(cnt, :) = theta;
end
traj = traj(1:cnt, :);
if cnt >= iter_max
    fprintf("Exit due to # of iterations exceed maximum allowable (%d).\n",iter_max)
end
end

function S = m_logm(T)
    [w, ang] = m_rotm2axang(T(1:3,1:3));
    if ang ~= 0
        % TODO: verify this
        v = (eye(3)/ang-skew(w)/2+(1/ang-cot(ang/2)/2)*skew(w)^2)*T(1:3,4);
        S = [w; v]*ang;
    else
        v = T(1:3,4);
        S = [0; 0; 0; v];
    end
end

function w_d  = grad_w(S, a, M)
    epsilon = 1e-6;
    n = length(a);
    J = J_body(S, a, M);
    w = sqrt(det(J*J'));
    w_d = zeros(n, 1);
    for ii = 1:n
        da = zeros(n, 1);
        da(ii) = epsilon;
        JJ = J_body(S, a+da, M);
        w_d(ii) = (sqrt(det(JJ*JJ'))-w)/epsilon;
    end
end