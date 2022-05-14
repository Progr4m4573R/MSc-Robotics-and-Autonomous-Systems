%Task 3 setup
a = csvread('a.csv')
b = csvread('b.csv')
x = csvread('x.csv')
y = csvread('y.csv')

nx = a - x
ny = b - y
nx_mean = mean(nx)
nx_std_dv = std(nx)

ny_mean = mean(ny)
ny_std_dv = std(ny)
figure
plot(x,y,'xb');
title('Plot of real x and y')
hold
%figure
plot(a,b,'+r');
title('Plot of noise a and b')

z = [a;b]
%Task 3 kalman tracking
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