u = csvread('u(1).csv')
v = csvread('v(1).csv')
x = csvread('x(1).csv')
y = csvread('y(1).csv')

nx = u - x
ny = v - y
nx_mean = mean(nx)
nx_std_dv = std(nx)

ny_mean = mean(ny)
ny_std_dv = std(ny)
figure
plot(x,y,'xb');
title('Plot of real x and y')
hold
%figure
plot(u,v,'+r');
title('Plot of noise u and v')

z = [u;v]
%Task 2
[px, py] = kalmanTracking(z)
figure
plot(px,py,'o');
title('Plot of Estimated px and py')
hold;

%Task 3 Evaluation
mean_pred1 = x - px
mean_pred2 = y - py
e = sqrt((mean_pred1.^2) + (mean_pred2.^2))
outpout = rms(e)
