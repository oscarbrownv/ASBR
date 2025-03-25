w = [0 0 1; 1 0 0; 1 0 0; 0 1 0; 1 0 0; 0 1 0]';
q = [0 0 0; 0 250 0; 0 250 770; 0 250 700; 0 1030 700; 0 1245 700]';
S = zeros(6, 6);
for ii = 1:6
    vi = -cross(w(:, ii), q(:, ii));
    S(:, ii) = [w(:, ii); vi];
end

x = sym("x", [6 1], "real");
J = simplify(J_space(S, x));

syms z [6 1] real
% Jw = simplify(J(1:3, :));
Jv = simplify(J(4:6, :));

% sols1 = solve(Jw * z == 0, z, "ReturnConditions", true);
sols2 = solve(det(Jv*Jv') == 0, "ReturnConditions", true);


