function [R,p] = correspondence(A,B)
% registration.m calculates the rigid transform between two point sets, i.e.
% solve for R and p such that R*A + p = B.  Implementation follows the
% quaternion-based absolute orientation method from Horn (1987).
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
G = [trace(H) D'; D H+H'-trace(H)*eye(3)];  % 4x4 Davenport matrix
[V,D] = eig(G);
R = m_quat2rotm(V(:,4)/norm(V(:,4)));  % eigenvector associated with max eigenvalue
p = b-R*a;
assert(m_isequal(det(R),1))

end