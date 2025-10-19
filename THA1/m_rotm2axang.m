function [ax, ang] = m_rotm2axang(R)
%M_ROTM2AXANG Convert a rotation matrix to axis-angle form.
%
%   [ax, ang] = M_ROTM2AXANG(R) interprets the 3x3 rotation matrix R as an
%   element of SO(3) and returns the equivalent axis-angle representation.
%   The axis vector ax is a unit direction describing the line of rotation,
%   while ang is the signed rotation magnitude in radians.  This mapping is
%   the logarithm map for SO(3) and is fundamental to screw theory and the
%   exponential coordinates used throughout the course.
%
%   The implementation follows the textbook formulas but highlights the two
%   singular configurations that appear in practice:
%     * Zero rotation (ang = 0) where the axis direction is arbitrary.
%     * 180 degree rotation (ang = pi) where the standard formula becomes
%       ill-conditioned and the axis must be extracted from the diagonal
%       elements of R.

ang = real(acos((trace(R)-1)/2));

if m_isequal(ang, 0)
    % When ang is zero, R is approximately the identity matrix and the axis
    % is undefined.  Returning the zero vector makes the downstream logic
    % aware that no rotation occurred.
    ax = [0; 0; 0];
elseif m_isequal(ang, pi)
    % Rotations of 180 degrees lead to numerical precision issues if the
    % generic skew-symmetric formula is used.  Instead we recover the axis by
    % taking square-roots of the diagonal entries, enforcing the correct
    % signs using the off-diagonal elements.  This mirrors the derivation of
    % eigenvectors for symmetric matrices.
    ax = sqrt(([R(1,1); R(2,2); R(3,3)] + 1)/2);
    ax(2) = sign(R(1,2) + 1e-16)*ax(2);
    ax(3) = sign(R(1,3) + 1e-16)*ax(3);
else
    % Generic case: use the skew-symmetric part of R to recover the axis.
    % Dividing by 2*sin(theta) corresponds to normalising the twist.
    ax = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]/(2*sin(ang));
end

end
