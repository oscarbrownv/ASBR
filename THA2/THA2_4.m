Rx = @(t) [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
Ry = @(t) [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
Rz = @(t) [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
theta = sym("t", [6 1], "real");
L = sym("L", "positive");

w = [0 0 1; 0 1 0; -1 0 0; -1 0 0; -1 0 0; 0 1 0;]';
q = [0 0 0; 0 0 0; 0 0 0; 0 L 0; 0 2*L 0; 0 3*L 0]';
S = sym(zeros(6, 6));
ww = sym(zeros(size(w)));
R = eye(3);
for ii = 1:6
    S(:, ii) = [w(:, ii); -cross(w(:, ii), q(:, ii))];
    ww(:, ii) = R*w(:, ii);
    R = R * m_axang2rotm(w(:, ii), theta(ii));
end

M = [eye(3) [0; L; 0]; zeros(1, 3), 1];
Js = J_space(S, theta);
Jb = J_body(S, theta, M);

t1 = 1;
t2 = pi/2;
t3 = 2;
t4 = 0.01;
t5 = 0;
t6 = 0;
L = 1;

A = double(subs(Js));
[U1, S1, V1] = svd(A);
B = double(subs(Jb));
[U2, S2, V2] = svd(B);

% Case 1: t2 = pi/2
% Case 3: t5 = -t4/2

