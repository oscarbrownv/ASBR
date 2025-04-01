function [q_final, traj] = J_transpose_kinematics(S, Tsd, M, q_init, options)
disp(options);
%if options are not given, set to these default settings for max iterations,
%tolerance, and alpha value
if nargin < 3
    options.max_iter = 1000;
    options.tol = 1e-3;
    options.alpha = 0.1;
%if only some options are given, unpopulated fields given default settings
else
    if ~isfield(options, 'max_iter')
        options.max_iter = 1000;
    end
    if ~isfield(options, 'tol')
        options.tol = 1e-3;
    end
    if ~isfield(options, 'alpha')
        options.alpha = 0.1;
    end
end

%starting joint configuration
q = q_init;
traj = zeros(options.max_iter, length(q_init));
traj(1, :) = q_init;

%for loop that iterates based on max_iter.
for iter = 1:options.max_iter
    %calculates the error between the desired pose and the current pose
    Tbd = FK_space(S, q, M)\Tsd;
    %calculates error vector through matrix logarithm
    err = m_logm(Tbd);
    %calculates the magnitude of the error vector
    err_norm = norm(err);
   
    %checks err_norm against user's tolerance. If less than user's
    %tolerance, break from for loop.
    if err_norm < options.tol
        fprintf('Converged in %d iterations. Final norm: %f\n', iter, err_norm);
        traj = traj(1:iter, :);
        break;
    end
    %compute Jacobian for delta_q equation
    J = J_body(S, q, M);

    %calculate the change in joint configurations
    err(4:6) = err(4:6) * 30;
    delta = J' * err;
    alpha = (norm(delta) / norm(J*delta))^2;

    %add delta_q to q for joint configuration of next iteration
    q = mod(q + alpha*delta, 2*pi);
    traj(iter+1,:) = q;
end

%save final joint configuration from for loop
q_final = q;

%Statement to end script when max iterations is reached and magnitude of
%error vector is still above tolerance. 
if iter == options.max_iter && err_norm >= options.tol
    warning ('Maximum iterations reached without convergence, Final error norm: %f,', err_norm);
end

function S = m_logm(T)
    [w, ang] = m_rotm2axang(T(1:3,1:3));
    if ang ~= 0
        v = (eye(3)/ang-skew(w)/2+(1/ang-cot(ang/2)/2)*skew(w)^2)*T(1:3,4);
        S = [w; v]*ang;
    else
        v = T(1:3,4);
        S = [0; 0; 0; v];
    end
end
end
