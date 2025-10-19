function Q = m_rotm2quat(R)
%M_ROTM2QUAT Convert a rotation matrix into a quaternion.
%
%   Q = M_ROTM2QUAT(R) packages the rotation R into a unit quaternion stored
%   as [w; x; y; z].  Quaternions are ideal for blending orientations or
%   passing data to simulation environments such as ROS or Unity.  This helper
%   highlights how all of these representations connect: we first obtain the
%   axis-angle description using the logarithm map and then convert that into
%   the four-parameter quaternion form.

[ax, ang] = m_rotm2axang(R);
Q = [cos(ang/2); ax*sin(ang/2)];

end