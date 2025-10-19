function ang = m_rotm2zyz(R)
%M_ROTM2ZYZ Convert a rotation matrix to ZYZ Euler angles.
%
%   ang = M_ROTM2ZYZ(R) expresses the SO(3) element R using the classical ZYZ
%   Euler sequence.  ZYZ angles are the natural parameterisation that arise
%   in the derivation of spherical wrist kinematics and are therefore a good
%   complement to the roll-pitch-yaw function provided elsewhere in this
%   repository.
%
%   The middle rotation (about the y-axis of the intermediate frame) plays a
%   special role.  When it is zero or pi the representation becomes singular
%   because the first and final rotations act about the same physical axis.
%   In those cases we manually resolve the redundancy by setting the first
%   angle to zero and folding all rotation into the last angle.

b = atan2(sqrt(R(3,1)^2+R(3,2)^2), R(3,3));
if m_isequal(sin(b), 0)
    % Singularity: the frame is aligned such that only a single axis is
    % effective.  Choosing a = 0 keeps the output well-defined while
    % preserving the overall transformation.
    c = atan2(R(2,1), R(1,1)/R(3,3));
    ang = [0; b; c];
else
    % Generic case: solve for the two outer rotations using the corresponding
    % columns of R.  The signs in the atan2 calls match the convention used in
    % modern robotics texts.
    a = atan2(R(2,3), R(1,3));
    c = atan2(R(3,2), -R(3,1));
    ang = [a; b; c];
end

end
