[q_Robot_config, q_camera_config, t_Robot_config, t_camera_config] = data_quaternion();

numConfigs = size(q_Robot_config, 1);
numPairs = numConfigs - 1;

qr_relative = zeros(numPairs,4);
qc_relative = zeros(numPairs, 4);

for i =1:numPairs
qr_current = q_Robot_config(i,:)/ norm(q_Robot_config(i,:));
qr_next = q_Robot_config(i+1,:)/ norm(q_Robot_config(i+1,:));
cr_current = q_camera_config(i,:)/ norm(q_camera_config(i,:));
cr_next = q_camera_config(i+1,:)/ norm(q_camera_config(i+1,:));

qr_relative(i,:) = quatmultiply(quatconj(qr_current),(qr_next));
qc_relative(i,:) = quatmultiply(quatconj(qc_current),(qc_next));

end

M = [];

for i = 1:numPairs
    L = LeftQuatM(q_robot_relative(i,:));
    R = RightQuatM(q_cam_relative(i,:));

    M = [M; (L-R)];
end


[U, S, V] = svd(M);
q_X = V(:, end);
q_X = q_X' / norm(q_X);

R_X = m_quat2rotm(q_X);

T_robot = zeros(4,4,numConfigs);
T_cam = zeros(4,4,numConfigs);

for i = 1:numConfigs
    qr = q_Robot_config(i,:)/norm(q_Robot_config(i,:));
    qc = q_camera_config(i,:)/norm(q_camera_config(i,:));

    R_robot_current = m_quat2rotm(qr);
    R_cam_current = m_quat2rotm(qc);

    T_robot(:,:,i) = [R_robot_current, t_Robot_config(i,:)'; 0 0 0 1];
    T_cam(:,:,i) = [R_cam_current, t_camera_config(i,:)'; 0 0 0 1];

end


A = zeros(4,4,numPairs);
B = xeros(4,4,numPairs);

for i = 1:numPairs
    A(:,:,i) = T_robot(:,:,i) \ T_robot(:,:,i+1);
    B(:,:,i) = T_cam(:,:,i) \ T_cam(:,:,i+1);
end

A_trans = [];
B_trans = [];

for i = 1:numPairs
    R_A_current = A(1:3, 1:3, i);
    t_A_current = A(1:3,4,i);
    t_B_current = B(1:3,4,i);

    A_trans = [A_trans; (R_A_current - eye(3))];
    B_trans = [B_trans; (R_X * t_B_current - t_A_current)];

end

t_X = A_trans \ B_trans;

X = [R_X, t_X; 0 0 0 1];

disp('Estimated Rotation in Quaternion form: ');
disp(q_X);
disp('Estimated Rotation Matrix R_X: ');
disp(R_X);
disp('Estimated Translation t_X: ');
disp(t_X);
disp('Estimated Transformation from camera to robot end-effector, X: ');
disp(X);
