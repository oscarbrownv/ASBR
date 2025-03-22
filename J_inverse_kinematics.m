function theta = J_inverse_kinematics(S, Tsd, M, theta0)
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

theta = theta0;
Tbd = FK_space(S, theta, M, false)\Tsd;
s = m_logm(Tbd);
while norm(s) > tol
    theta = theta + real(pinv(J_body(S, theta, M)))*s;
    Tbd = FK_space(S, theta, M, false)\Tsd;
    s = m_logm(Tbd);
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