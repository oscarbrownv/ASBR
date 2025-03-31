function A = Ad(g)
    R = g(1:3, 1:3);
    p = g(1:3, 4);
    A = [R zeros(3, 3); skew(p)*R R];
end