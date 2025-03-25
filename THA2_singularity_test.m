% Screw axes for SCARA robot
l1 = 1; l2 = 1;
S = [0 0 1 0 0 0; 0 0 1 l1 0 0; 0 0 1 l1+l2 0 0; 0 0 0 0 0 1]';
sing = singularity(S);
disp("Singularities:")
disp(~sing)

% Found singularity x2 == 0 or pi, x1 = pi/2 or 3pi/2

%% Non singular case
x = [pi/3; pi/4; 0; 0];
J = J_space(S, x);
assert(rank(J) == 4)

%% Singularity x1 == pi/2
x = [pi/2; 0; 0; 0];
J = J_space(S, x);
assert(rank(J) < 4)

%% Singularity x1 == 3pi/2
x = [3*pi/2; 0; 0; 0];
J = J_space(S, x);
assert(rank(J) < 4)

%% Singularity x2 == 0
x = [0; 0; 0; 0];
J = J_space(S, x);
assert(rank(J) < 4)

%% Singularity x2 == pi
x = [0; pi; 0; 0];
J = J_space(S, x);
assert(rank(J) < 4)

