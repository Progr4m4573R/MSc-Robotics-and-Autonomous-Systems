%I = imread('cameraman.tif');
%figure
%imshow(I);

%https://www.mathworks.com/help/coder/ug/edge-detection-on-images.html
% [X,map] = imread('cameraman.tif');
% figure
% imshow(X,map);
% I_gray=rgb2gray(X);
% figure
% imshow(I_gray);
% C=im2double(I_gray);
% % create an empty image with the same size of the input image
% I_out=zeros(size(C)); 
%https://uk.mathworks.com/matlabcentral/answers/24654-sobel-edge-detection-implementation-problem
original_Image = imread('mandril_color.tif');
figure
imshow(original_Image);


orig_gray = rgb2gray(original_Image);
figure
imshow(orig_gray);
orig_dub = im2double(orig_gray); 

gx=[-1 0 1;-2 0 2;-1 0 1];
d=conv2(orig_dub,gx);
gy=[1 2 1;0 0 0;-1 -2 -1];
e=conv2(orig_dub,gy);

G=zeros(size(orig_dub)); 

[m, n]=size(e);
for i=1:m
    for j=1:n
        G(i,j)=uint8(sqrt(d(i,j)*d(i,j)+e(i,j)*e(i,j)));
   end
end
figure;
imshow(G);