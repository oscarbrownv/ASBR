KukaQuantec;

q = [1; 1; 0; 1; 1; 1];
F = FK_space(S,q,M);
ub = deg2rad([180; 45; 150; 350; 125; 350]);
lb = deg2rad([-180; -145; -130; -350; -125; -350]);
tool = [0.1; 0; 0];
tol = 0.003;
tool_w = F*[tool; 1];
tool_w = tool_w(1:3);
z = F*[0; 0; 1; 1];
z = z(1:3);
goal = tool_w(1:3) + [-0.002; 0.001; 0.001];
dq1 = stay_near_point(S,tool_w,q,ub,lb,goal,tol);
dq2 = stay_near_point_with_orientation_control(S,tool_w,z,q,ub,lb,goal,tol,0.5,0.5);

q1 = q+dq1;
F1 = FK_space(S,q1,M);
tool_w1 = F1*[tool; 1];
z1 = F1*[0; 0; 1; 1];
z1 = z1(1:3)/norm(z1(1:3));
norm(tool_w1(1:3)-goal)
norm(cross(z,z1))
draw(tool_w,tool_w1,z,z1)

q2 = q+dq2;
F2 = FK_space(S,q2,M);
tool_w2 = F2*[tool; 1];
z2 = F2*[0; 0; 1; 1];
z2 = z2(1:3)/norm(z2(1:3));
norm(tool_w2(1:3)-goal)
norm(cross(z,z2))
draw(tool_w,tool_w2,z,z2)


function draw(p1,p2,z1,z2)
    figure;
    plot3([p1(1),p2(1)],[p1(2),p2(2)],[p1(3),p2(3)],'LineWidth',3,'DisplayName',"Path");
    hold on
    plot3([p1(1),p1(1)-z1(1)],[p1(2),p1(2)-z1(2)],[p1(3),p1(3)-z1(3)],"--",'LineWidth',3,'Color','r','DisplayName',"Initial Tool Tip Orientation");
    hold on
    plot3([p2(1),p2(1)-z2(1)],[p2(2),p2(2)-z2(2)],[p2(3),p2(3)-z2(3)],"--",'LineWidth',3,'Color','g','DisplayName',"End Tool Tip Orientation");
    hold on
    scatter3(p1(1),p1(2),p1(3),108,'filled','r','DisplayName',"start")
    hold on
    scatter3(p2(1),p2(2),p2(3),108,'filled','g','DisplayName',"goal")
    legend()
end