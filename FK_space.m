function T = FK_space(Ss, as, M, show_plot)
% FK_space.m calculates the space form forward kinematics (FK).
% Assume ||w|| == 1.
% 
% Inputs:
%   Ss: i-th column is the i-th screw axis [wi; vi]
%   as: i-th value is the angle corresponding to the i-th screw axis
%   M : initial configuration
%
% Outputs:
%   T : the transformation matrix representing the pose of the end effector
%       in the world coordinate frame

arguments
    Ss (6,:) double
    as (1,:) double
    M (4,4) double
    show_plot (1,1) logical 
end

% Check that the dimension of screw axes is correct
assert(size(Ss, 1) == 6)

% Check that the length of the screw axes and the angles match
assert(size(Ss, 2) == length(as))
N = length(as);

T = M;
for ii = N:-1:1
    T = exp_twist(Ss(:, ii), as(ii)) * T;
end

if show_plot
    figure;
    x = zeros(3, 1);
    plotTransforms(x', [1 0 0 0]);
    hold on
    Adg = eye(6);
    prev_x = x;
    for ii = 1:N
        S = Adg*Ss(:,ii);
        if ~m_isequal(S(1:3), zeros(size(S(1:3))))
           x = cal_q(S);
        else
           x = prev_x;
        end
        plotScrewAxis(x, S(1:3));
        Adg = Adg * Ad(exp_twist(Ss(:, ii), as(ii)));
        plotSegment(prev_x, x, "LineWidth", 2, "Color", "black");
        prev_x = x;
    end
    plotSegment(prev_x, T(1:3,4), "LineWidth", 2, "Color", "black");
    plotTransforms(T(1:3,4)', m_rotm2quat(T(1:3,1:3))')
end

end

function q = cal_q(S)
    w = S(1:3);
    v = S(4:6);
    a = cross(w, v);
    proj_wa = dot(a,w)/norm(w)*w;
    q = a - proj_wa;
end