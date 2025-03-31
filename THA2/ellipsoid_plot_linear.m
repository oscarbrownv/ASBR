function ellipsoid_plot_linear(Jv)
% ellipsoid_plot_linear.m plot the manipulability ellipsoids for the
% linear velocities.
%
% Inputs:
%   Jv: the bottom three rows of the Jacobian of the manipulator

A = Jv * Jv';
[V, D] = eig(A);
[X, Y, Z] = ellipsoid(0, 0, 0, D(1,1), D(2,2), D(3,3));
s = surf(X, Y, Z, "FaceColor", "none");
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])

% TODO: check if the axes are correct
ang = rad2deg(m_rotm2zyz(V));
rotate(s, [0 0 1], ang(1));
rotate(s, [0 1 0], ang(2));
rotate(s, [0 0 1], ang(3));

end