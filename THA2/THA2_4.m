% THA2_4.m
% Symbolic exploration of the space and body Jacobians for a six-axis arm.
% The script constructs the screw axes for each joint, builds the analytical
% Jacobians, and then studies their singular values through the singular value
% decomposition (SVD).  The hard-coded joint configurations at the end mirror
% the cases discussed in the coursework notes so that the numerical values
% line up with the theory.

theta = sym("t", [6 1], "real");
L = sym("L", "positive");

% Screw axes expressed in the space frame.  Each column corresponds to the
% angular velocity direction of a joint.  Repeated axes reflect parallel
% shoulder/elbow joints in the model manipulator.
w = [0 0 1; 0 1 0; -1 0 0; -1 0 0; -1 0 0; 0 1 0;]';
q = [0 0 0; 0 0 0; 0 0 0; 0 L 0; 0 2*L 0; 0 3*L 0]';
S = sym(zeros(6, 6));
ww = sym(zeros(size(w)));
R = eye(3);
for ii = 1:6
    % Assemble the twist [w; v] for joint ii.  The linear velocity component v
    % is obtained from the moment of the axis through the point q.
    S(:, ii) = [w(:, ii); -cross(w(:, ii), q(:, ii))];
    % Record the rotated axis so that the body-frame Jacobian can be built by
    % propagating the rotation matrix forward.
    ww(:, ii) = R*w(:, ii);
    R = R * m_axang2rotm(w(:, ii), theta(ii));
end

% Home configuration of the end-effector and Jacobian expressions.
M = [eye(3) [0; L; 0]; zeros(1, 3), 1];
Js = J_space(S, theta);
Jb = J_body(S, theta, M);

% Particular joint values used in the tutorial exercises.
t1 = 1;
t2 = pi/2;
t3 = 2;
t4 = 0.01;
t5 = 0;
t6 = 0;
L = 1;

% Evaluate Jacobians and study their singular values to identify near-
% singular configurations.
A = double(subs(Js));
[U1, S1, V1] = svd(A);
B = double(subs(Jb));
[U2, S2, V2] = svd(B);

% Case 1: t2 = pi/2
% Case 3: t5 = -t4/2
