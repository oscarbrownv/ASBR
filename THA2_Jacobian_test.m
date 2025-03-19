%% SCARA robot spatial Jacobian
l1 = 2;
l2 = 1;
a = [pi/3; pi/4; 0; 0]; % Rotational angles
S = [0 0 1 0 0 0;
      0 0 1 l1 0 0;
      0 0 1 l1+l2 0 0;
      0 0 0 0 0 1]';

J = J_space(S, a);
J_ref = [0 0 0 0;
         0 0 0 0;
         1 1 1 0;
         0 l1*cos(a(1)) l1*cos(a(1))+l2*cos(a(1)+a(2)) 0;
         0 l1*sin(a(1)) l1*sin(a(1))+l2*sin(a(1)+a(2)) 0;
         0 0 0 1;];

% Check if the results are correct
assert(m_isequal(J,J_ref))