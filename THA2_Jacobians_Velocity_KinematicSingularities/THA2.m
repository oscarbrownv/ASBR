% THA2.m
% Simple example that constructs a two-link manipulator using MATLAB's
% robotics toolbox.  The goal is to illustrate how modified Denavit-Hartenberg
% parameters and geometric Jacobians are specified programmatically.  This is
% a lightweight script that can be run interactively while revising key
% kinematics concepts.

robot = robotics.RigidBodyTree('DataFormat', 'column'); % robot configuration as column vector

% add first body (fixed, with an offset from ICS)
body1 = robotics.RigidBody('body1');
body1.Joint = robotics.Joint('joint1', 'revolute');
T = trvec2tform([-0.5, 0 0.2])*eul2tform([pi/2 0 0]);
body1.Joint.setFixedTransform(T);
robot.addBody(body1, robot.BaseName);

% add second body (revolute joint)
body2 = robotics.RigidBody('body2');
body2.Joint = robotics.Joint('joint2', 'revolute');
body2.Joint.setFixedTransform([0.3 -pi/3 0.1 0], 'mdh');
robot.addBody(body2, 'body1');

% Query the Jacobian at the zero configuration to reveal how joint motion
% translates into spatial velocity of the end-effector.
Jac = robot.geometricJacobian([0; 0], 'body2')
