clear;
clc;
% Hand-eye calibration walkthrough using quaternion-based relative motions.
% The script replays the least-squares derivation from lecture: compute
% AX=XB with SVD for rotation and a linear solve for translation, then report
% residuals for each pose pair.
% import data
[q_Robot_config, q_camera_config, t_Robot_config, t_camera_config] = data_quaternion();
% declare variables for number of configurations and number of pose pairs
numConfigs = size(q_Robot_config, 1);
numPairs = numConfigs - 1;

% preallocate relative motions of robot and camera between configurations
q_robot_rel = zeros(numPairs, 4);
q_camera_rel = zeros(numPairs, 4);
% Compute relative motion between each consecutive configuration of the
% camera and robot.  Using relative quaternions avoids numerical drift from
% accumulating absolute poses.
for i = 1:numPairs
    % Relative quaternion for robot
    q_robot_rel(i,:) = quatmultiply(quatconj(q_Robot_config(i,:)), q_Robot_config(i+1,:));
    
    % Relative quaternion for camera
    q_camera_rel(i,:) = quatmultiply(quatconj(q_camera_config(i,:)), q_camera_config(i+1,:));

    % Check for sign discrepancies.  Enforcing a positive scalar component
    % keeps the quaternions on the same hemisphere.
    if q_robot_rel(i,1) < 0
        q_robot_rel(i,:) = -q_robot_rel(i,:);
    end
    if q_camera_rel(i,1) < 0
        q_camera_rel(i,:) = -q_camera_rel(i,:);
    end
    % Normalize to guard against accumulation of floating point error.
    q_robot_rel(i,:) = q_robot_rel(i,:) / norm(q_robot_rel(i,:));
    q_camera_rel(i,:) = q_camera_rel(i,:) / norm(q_camera_rel(i,:));
end

M = [];
% create the M matrix that stacks all relative motion constraints

for i = 1:numPairs
    %for each iteration, separates the scalar from the angular portion of
    %the quaternion
    qA = q_robot_rel(i,:);
    qB = q_camera_rel(i,:);
    sA = qA(1); 
    vA = qA(2:4);
    sB = qB(1); 
    vB = qB(2:4);
    
    % Build M(qA,qB) to be used in svd()
    % [ (sA - sB), -(vA - vB);
    %   (vA - vB)', ((sA - sB)*eye(3) + skew(vA + vB)) ]

    M_current = [
        (sA - sB), -(vA - vB);
        (vA - vB)', ( (sA - sB)*eye(3)) + skew(vA+vB)
    ];

    M = [M; M_current];
end

% Decompose M to extract the quaternion corresponding to the smallest
% singular value (classic quaternion hand-eye solution).
[U, S, V] = svd(M);
%assigning q_X the value of the final column in V, corresponding to
%smallest value in U
q_X = V(:, end);
q_X = q_X' / norm(q_X);
%converting normalize q_X to make R_X for later use
R_X = m_quat2rotm(q_X');
% Preallocate transformation matrices for the raw robot/camera poses.
T_robot = zeros(4,4,numConfigs);
T_cam = zeros(4,4,numConfigs);

%Build transformation matrices for each pose of camera and end effector
for i = 1:numConfigs
    qr = q_Robot_config(i,:)/norm(q_Robot_config(i,:));
    qc = q_camera_config(i,:)/norm(q_camera_config(i,:));

    R_robot_current = m_quat2rotm(qr);
    R_cam_current = m_quat2rotm(qc);

    T_robot(:,:,i) = [R_robot_current, t_Robot_config(i,:)'; 0 0 0 1];
    T_cam(:,:,i) = [R_cam_current, t_camera_config(i,:)'; 0 0 0 1];

end


A = zeros(4,4,numPairs);
B = zeros(4,4,numPairs);
% Create A and B matrices representing relative transforms between poses.
for i = 1:numPairs
    A(:,:,i) = inv(T_robot(:,:,i)) * T_robot(:,:,i+1);
    B(:,:,i) = inv(T_cam(:,:,i)) * T_cam(:,:,i+1);
end

A_trans = [];
B_trans = [];
% Solve for the translational component of X via linear least squares.
for i = 1:numPairs
    R_A_current = A(1:3, 1:3, i);
    t_A_current = A(1:3,4,i);
    t_B_current = B(1:3,4,i);

    A_trans = [A_trans; (R_A_current - eye(3))];
    B_trans = [B_trans; (R_X * t_B_current - t_A_current)];

end
% compute translational portion of transformation
t_X = A_trans \ B_trans;

%put rotational and translational parts together to make X transformation
X = [R_X, t_X; 0 0 0 1];

% Display results for quick inspection.
disp('Estimated Rotation in Quaternion form: ');
disp(q_X);
disp('Estimated Rotation Matrix R_X: ');
disp(R_X);
disp('Estimated Translation t_X: ');
disp(t_X);
disp('Estimated Transformation from camera to robot end-effector, X: ');
disp(X);

%% Test script to determine how well the transformation X fits the equation AX = XB.
% Residuals here give confidence that the quaternion/translation solves are
% consistent across all pose pairs.

error_norm = zeros(numPairs, 1);

for i = 1:numPairs
    residual = A(:,:,i) * X - X * B(:,:,i);

    ref_norm_A = norm(A(:,:,i) * X, 'fro');
    ref_norm_B = norm(X * B(:,:,i), 'fro');
    ref_norm = (ref_norm_A + ref_norm_B) / 2;


    error_norm(i) = norm(residual, 'fro');

    percent_error = (error_norm(i)/ref_norm) * 100;

    fprintf('Error norm for pair %d: %e\n', i ,percent_error);

end

rmserror = sqrt(mean(error_norm.^2));
fprintf('Overall RMS error: %e\n', rmserror);







% Test case: Find transformation C (fixed point in space to base of robot)
% based on S1, X and E1 (should be S1 * X * E1)
%Test first five, second five, and compare to all 10 cases