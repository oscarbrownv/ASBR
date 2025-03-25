function singularity()
% singularity.m calculates singular configurations of the robot.
% At a singular configuration the Jacobian drops rank.
% Hence, the Jacobian fails to be invertible at singular points and
% becomes unable to move in certain directions.

% TODO: use symbolic expression for the Jacobian and solve for det(JJ')=0
l1 = 2;
l2 = 1;
a = sym("a", [4 1]); % Rotational angles
S = [0 0 1 0 0 0;
     0 0 1 l1 0 0;
     0 0 1 l1+l2 0 0;
     0 0 0 0 0 1]';

J = J_space(S, a);

end