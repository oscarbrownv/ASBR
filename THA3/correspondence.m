function [R,p] = correspondence(A,B)
% registration.m calculates the transformation between two sets, i.e.
% R*A+p=B.
% 
% Inputs:
%   A: 3xN matrix
%   B: 3xN matrix
%
% Outputs:
%   R: rotation matrix
%   p: translation vector

a = mean(A,2);
b = mean(B,2);
H = (A-a)*(B-b)';
D = [H(2,3)-H(3,2); H(3,1)-H(1,3); H(1,2)-H(2,1)];
G = [trace(H) D'; D H+H'-trace(H)*eye(3)];
[V,D] = eig(G);
R = m_quat2rotm(V(:,4));
p = b-R*a;
assert(m_isequal(det(R),1))

end