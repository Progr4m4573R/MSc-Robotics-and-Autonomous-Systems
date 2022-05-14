clear all; close all; clc
%Task 2 Part 1 select features for radius and direction

%https://uk.mathworks.com/help/images/create-image-histogram.html
imgpia = imread('ImgPIA.jpg');
figure
imshow(imgpia)
%set radius and angle
radius = 100;%chosen radius
angle = 360;%total number of angles

img = double(imgpia);
[rows, columns] = size(imgpia);
cy = round(rows/2);
cx = round(cols/2);

if exist('radius','var') == 0
    radius = min(round(rows/2),round(cols/2))-1;
end

if exist('angle','var') == 0
    angle = 360;
end
polarcord_img=zeros(); %from cartesian coordinate to polar
i=1;
%this loop performs the interpolation for radius and theata angle we use 0
for r = 0:radius
    j = 1;
    for a=0:2*pi/angle:2*pi-2*pi/angle
    polarcord_img(i,j) = img(cy+round(r*sin(a)),cx+round(r*cos(a)));
    j = j+1;
    end
    i = i+1;
end
%after converting image
%%Spectral feature calculation
S_r_theta = fft.(polarcord_img);
%Plot for theta
for theta=1:360
    S_theta(theta) = sum ((abs(S_r_theta(:,theta))));
end
figure(2)
bar(S_theta)
xlabel('theta')
ylabel('feature value')
%Task 2 Part 2
%co-currence matrix
output = make_co_occurence_matrix(ImgPIA)


