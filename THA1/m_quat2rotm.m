function R = m_quat2rotm(Q)
%M_QUAT2ROTM Convert a unit quaternion into a rotation matrix.
%
%   R = M_QUAT2ROTM(Q) maps the quaternion Q = [w; x; y; z] into its
%   equivalent 3x3 rotation matrix.  Quaternions are a numerically stable way
%   to represent orientations without suffering from the singularities of
%   Euler angles, so this helper is a common bridge when interfacing with
%   software libraries that operate on matrices.
%
%   The conversion uses the well-known relationship between quaternions and
%   axis-angle parameters: the scalar part encodes half of the rotation angle
%   and the vector part encodes the rotation axis scaled by sin(theta/2).

assert(m_isequal(norm(Q), 1), "Must be unit quaternion")

ang = 2*acos(Q(1));
if ang ~= 0
    % Recover the rotation axis from the vector portion.  Dividing by
    % sin(ang/2) normalises the axis because the quaternion stores it scaled
    % by sin(theta/2).
    ax = Q(2:4)/sin(ang/2);
else
    % Identity rotation: there is no preferred axis so we return zero.
    ax = [0; 0; 0];
end

R = m_axang2rotm(ax, ang);

end