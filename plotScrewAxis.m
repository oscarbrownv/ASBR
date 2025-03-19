function plotScrewAxis(q, s)
    l = 1;
    P = [q-s*l q+s*l];
    plot3(P(1,:), P(2,:), P(3,:), "--", "LineWidth", .5, "Color", "black", "DisplayName", "Screw axis");
    hold on
    scatter3(q(1), q(2), q(3), 100, "filled", "MarkerFaceColor", [1, 0.5, 0], "DisplayName", "Point on screw axis")
    hold on
end