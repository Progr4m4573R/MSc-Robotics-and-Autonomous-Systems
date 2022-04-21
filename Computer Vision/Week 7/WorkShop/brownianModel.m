function [x,y] = brownianModel()
% generate the 2D trajectory of a point moving according to a
% Brownian motion model
% initial position vectors
x = zeros(1000, 1);
y = zeros(1000, 1);
% noise vectors
nx = 0.5 * randn(1000, 1);
ny = 0.5 * randn(1000, 1);
% generate trajectory
    for i = 2:1000
    x(i) = x(i-1) + nx(i);
    y(i) = y(i-1) + ny(i);
    end
end