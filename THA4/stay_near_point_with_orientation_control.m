function dq = stay_near_point_with_orientation_control(S,t,z,q,ub,lb,goal,tol,zeta,eta)
% stay_near_point_with_orientation_control.m calculates small change of 
% angles that minimizes the distance between the tool tip and the goal.
% 
% Inputs:
%   S: i-th column is the i-th screw axis described in space frame [wi; vi]
%   t: coordinate of tool tip in world frame
%   z: tool axis in world frame
%   q: i-th value is the angle corresponding to the i-th screw axis
%   ub: upper bound joint limits
%   lb: lower bound joint limits
%   goal: goal coordiantes [x; y; z]
%   tol: tolerance
%   zeta: weight factor for translation term
%   eta: weight factor for orientation term
%
% Outputs:
%   dq: joint movements

J = J_space(S,q);
function cost = objective(dq)
    twist = J*dq;
    alpha = twist(1:3);
    epsilon = twist(4:6);
    cost = zeta*norm(skew(alpha)*t+epsilon+t-goal)^2+eta*norm(skew(alpha)*z)^2+norm(dq)^2;
    cost = zeta*norm(skew(alpha)*t+epsilon+t-goal)^2+eta*norm(skew(alpha)*z)^2;
end
function [c,ceq] = constraints(dq)
    twist = J*dq;
    alpha = twist(1:3);
    epsilon = twist(4:6);
    c = norm(skew(alpha)*t+epsilon+t-goal)-tol;
    ceq = [];
end

dq = fmincon(@objective,q,[],[],[],[],lb-q,ub-q,@constraints);
end