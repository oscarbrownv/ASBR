function qdot = redundancy_resolution(ve, S, a)
% redundancy_resolution.m solves the problem ve = J * qdot while maximizing
% the manipulaibility measure and moving away from singularities.
% 
% Inputs:
%   ve: desired end effector velocity (twist) expressed in body frame
%   S: i-th column is the i-th screw axis [wi; vi]
%   a: i-th value is the angle corresponding to the i-th screw axis
%
% Outputs:
%   theta_d: joint velocity

n = length(a);
k0 = 1;
q0_d = k0*grad_w(S, a);
J = J_space(S, a);
qdot = pinv(J)*ve + (eye(n)-pinv(J)*J)*q0_d;
end


function w_d  = grad_w(S, a)
    epsilon = 1e-6;
    n = length(a);
    J = J_space(S, a);
    w = sqrt(det(J*J'));
    w_d = zeros(n, 1);
    for ii = 1:n
        da = zeros(n, 1);
        da(ii) = epsilon;
        JJ = J_space(S, a+da);
        w_d(ii) = (sqrt(det(JJ*JJ'))-w)/epsilon;
    end
end