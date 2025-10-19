function ang = m_rotm2rpy(R)
%M_ROTM2RPY Convert a rotation matrix to roll-pitch-yaw angles.
%
%   ang = M_ROTM2RPY(R) returns the ZYX Euler angle sequence (often referred
%   to as roll, pitch, and yaw) that realises the rotation matrix R.  The
%   function is written explicitly rather than relying on MATLAB's Robotics
%   Toolbox so that the intermediate steps can be discussed when presenting
%   coursework.
%
%   The key geometric idea is that pitch corresponds to the rotation about
%   the intermediate y-axis.  When the pitch angle approaches +/-90 degrees
%   the representation becomes singular (gimbal lock) and the remaining
%   degrees of freedom collapse.  The branching logic below isolates this
%   special case and chooses a consistent solution so that the function can
%   be used safely in demonstrations.

p = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
if m_isequal(cos(p), 0)
    % When cos(p) is zero the configuration is in gimbal lock.  We select a
    % feasible yaw angle that preserves continuity of motion and set roll to
    % zero since it is not identifiable.
    y = atan2(-R(1,2)/R(3,1), R(2,2));
    ang = [0; p; y];
else
    % Regular configuration: recover roll and yaw independently by looking at
    % rotations around the body-fixed x and z axes respectively.
    r = atan2(R(2,1), R(1,1));
    y = atan2(R(3,2),R(3,3));
    ang = [r; p; y];
end

end
