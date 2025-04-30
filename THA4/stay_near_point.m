function dq = stay_near_point(S,t,q,ub,lb,goal,tol)
% stay_near_point.m calculates small change of angles that minimizes the
% distance between the tool tip and the goal.
% 
% Inputs:
%   S: i-th column is the i-th screw axis described in space frame [wi; vi]
%   q: i-th value is the angle corresponding to the i-th screw axis
%   tool: coordinate of tool tip in world frame
%   ub: upper bound joint limits
%   lb: lower bound joint limits
%   goal: goal coordiantes [x; y; z]
%   tol: tolerance
%
% Outputs:
%   dq: joint movements

J = J_space(S,q);
function cost = objective(dq)
    twist = J*dq;
    alpha = twist(1:3);
    epsilon = twist(4:6);
    cost = norm(skew(alpha)*t+epsilon+t-goal)^2;
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