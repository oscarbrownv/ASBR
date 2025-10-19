function ellipsoid_plot_angular(Jw)
% ellipsoid_plot_angluar.m plot the manipulability ellipsoids for the
% angular velocities.
%
% Inputs:
%   Jw: the top three rows of the Jacobian of the manipulator

A = Jw * Jw';
[V, D] = eig(A);
[X, Y, Z] = ellipsoid(0, 0, 0, D(1,1), D(2,2), D(3,3));
s = surf(X, Y, Z, "FaceColor", "none");
title("ellipsoid (angular)")

% TODO: check if the axes are correct
ang = rad2deg(m_rotm2zyz(V));
rotate(s, [0 0 1], ang(1));
rotate(s, [0 1 0], ang(2));
rotate(s, [0 0 1], ang(3));

end