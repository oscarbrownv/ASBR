function plotSegment(a, b, varargin)
    x = [a(1) b(1)];
    y = [a(2) b(2)];
    z = [a(3) b(3)];
    plot3(x, y, z, varargin{:});
    hold on
end
