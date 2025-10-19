clear;
clc;
% Command-line driver for the product-of-exponentials formulation on the
% KUKA Quantec arm.  This mirrors the homework write-up and walks through the
% FK computation step-by-step, making it a nice reference when practicing the
% manual calculations.
%
% User inputs the joint angles of each joint (in degrees) and the script
% validates the ranges before converting to radians.
% error function catches angles out of range
theta1 = input('Axis 1 joint angle (between -180 and 180): ');
if theta1 < -180 || theta1 >180
    error('Axis 1 angle out of range!')
end

theta2 = input('Axis 2 joint angle (between -145 and 45): ');
if theta2 < -145 || theta2 > 45
    error('Axis 2 angle out of range!')
end

theta3 = input('Axis 3 joint angle (between -120 and 150): ');
if theta3 < -120 || theta3 > 150
    error('Axis 3 angle out of range!')
end

theta4 = input('Axis 4 joint angle (between -350 and 350): ');
if theta4 < -350 || theta4 > 350
    error('Axis 4 angle out of range!')
end

theta5 = input('Axis 5 joint angle (between -125 and 125): ');
if theta5 < -125 || theta5 > 125
    error('Axis 5 angle out of range!')
end

theta6 = input('Axis 6 joint angle (between -350 and 350): ');
if theta6 < -350 || theta6 > 350
    error('Axis 6 angle out of range!')
end

%combining thetas into one array
theta_degree = [theta1; theta2; theta3; theta4; theta5; theta6];
%Converting the array from degrees to radians
theta_radians = deg2rad(theta_degree);

%Axes of rotation of each joint at home position
w1 = [0;0;1];
w2 = [1;0;0];
w3 = [0;1;0];
w4 = [1;0;0];
w5 = [0;1;0];
w6 = [1;0;0];
%Combining axes of rotation into one array
w_all = [w1,w2,w3,w4,w5,w6];

% Points along axes of rotation (values taken from the DH layout in mm)
q1= [0;0;0];
q2=[250;0;500];
q3=[250;0;1270];
q4=[0;0;1200];
q5=[1030;0;1200];
q6=[0;0;1200];
%combining all points along axes of rotation into one array
q_all = [q1,q2,q3,q4,q5,q6];

% Compute the screw axes by combining each unit rotation axis with a point
% lying on that axis.
for i = 1:6
    wi = w_all(:,i);
    qi = q_all(:,i);
    v = -cross(wi,qi);
    screwaxes(:,i) = [wi;v];
end

%defining the home position matrix
R = eye(3);
p = [1245; 0; 1200];
M = [R, p;
    0,0,0,1];

%First value of T to start product of exponentials
T = eye(4);

%Iterates through each factor in the product of exponentials
for i = 1:length(theta_radians)
    T = T * expm(screw2se3(screwaxes(:,i)) * theta_radians(i));
end

%finally multiplies by the home matrix
T = T * M;

% Display the resulting transformation for review.
disp('The forward kinematics transformation matrix T is:');
disp(T);



% Local helper: convert a twist to an se(3) matrix so it can be exponentiated.
function se3matrix = screw2se3(V)
    w = V(1:3);
    v = V(4:6);
    se3matrix = [skew(w), v; 0 0 0 0];
end

%Creates skew symmetric matrix for each screw axis for screw2se3 function
function S = skew(v)
    S = [0 -v(3) v(2);
        v(3) 0 -v(1);
        -v(2) v(1) 0];
end
