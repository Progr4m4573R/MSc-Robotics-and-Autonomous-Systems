figure
imshow(imgpia)
%set radius and angle
radius = 350;%chosen radius(100,150,200,250,300,350)
angle = 360;%total number of angles
radius_values = [100,150,200,250,300,350];
img = double(imgpia);
[rows, columns] = size(imgpia);
cy = round(rows/2);
cx = round(columns/2);
% for all radius values if any are 0 then get minimum value from row and
% column of image as use that as the radius
for rad=1:numel(radius_values)
    if exist('radius','var') == 0
        radius_values(rad) = min(round(rows/2),round(columns/2))-1;
    end
end

if exist('angle','var') == 0
    angle = 360;
end
polarcord_img=zeros(); %from cartesian coordinate to polar
i=1;
%this loop performs the interpolation for radius and theata angle we use 0
for rad = 1:numel(radius_values)
    %-----pick a value from 1-7 to see the effect of different radius----
    rad = 1;
    %-----pick a value from 1-7 to see the effect of different radius----

    for r = 1:radius_values(rad)

        j = 1;
        for a=0:2*pi/angle:2*pi-2*pi/angle
        polarcord_img(i,j) = img(cy+round(r*sin(a)),cx+round(r*cos(a)));
        j = j+1;
        end
        i = i+1;
    end
end

%after converting image
%%Spectral feature calculation
S_r_theta = fft(polarcord_img);

%plot for given rad value 
S_r=zeros();
for r=1:radius_values(rad)
    S_r(r) = sum((abs(S_r_theta(r+1,:))));
end
figure(1)
%in order to get spectral feature i convert to fourier domain
bar(S_r)
xlabel('radius')
ylabel('feature value')
%Plot for theta=0:360
S_theta=zeros();
for theta=1:360
    S_theta(theta) = sum ((abs(S_r_theta(:,theta))));
end

figure(2)
bar(S_theta)
xlabel('theta')
ylabel('feature value')