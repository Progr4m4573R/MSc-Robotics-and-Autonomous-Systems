n1 = randn(1000,1); % Gaussian noise with mean=0 and std=1
n2 = 5 + randn(1000,1); % Gaussian noise with mean=5 and std=1
n3 = 0.2 * randn(1000,1);% Gaussian noise with mean=0 and std=0.2

mean(n1)
std(n1)
var(n1)
hist(n1, 50)
[x,y] = brownianModel
plot(x,y)
figure
plot(x)
figure
plot(y)
%Task 3
[x,vx,y,vy] = cvModel
plot(x,y)
figure
plot(x)
figure
plot(y)
figure
plot(vx)
figure
plot(vy)