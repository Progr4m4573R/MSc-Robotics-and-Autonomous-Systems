function [x, vx, y, vy] = cvModel()
% generate the 2D trajectory and velocity of a point moving
% according to a Constant Velocity motion model
dt = 0.033; % time interval
% initial position and velocity vectors
x = zeros(1000, 1); % x position
vx = zeros(1000, 1); % x velocity
y = zeros(1000, 1); % y position
vy = zeros(1000, 1); % y velocity
% noise vectors
nx = 0.5 * randn(1000, 1);
nvx = 0.1 * randn(1000, 1);
ny = 0.5 * randn(1000, 1);
nvy = 0.1 * randn(1000, 1);
% generate trajectory
for i = 2:1000
    x(i) = x(i-1) + vx(i-1)*dt + nx(i);
    vx(i) = vx(i-1) + nvx(i);
    y(i) = y(i-1) + vy(i-1)*dt + ny(i);
    vy(i) = vy(i-1) + nvy(i);
    end
end
