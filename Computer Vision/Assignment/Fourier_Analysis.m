
%https://homepages.inf.ed.ac.uk/rbf/HIPR2/fourier.htm - helpful understanding
%https://uk.mathworks.com/help/matlab/ref/fft2.html
%https://uk.mathworks.com/help/matlab/ref/angle.html
%code based on Computer vision workshop 8
function[]=Fourier_Analysis(img)
%Prepare image
%-----------------------------------------------
grey = rgb2gray(img);
f = grey;
%------------------------------------------------
%rotate the image to 90 degree
rotatedF = imrotate(f,90);
%create the fourier transform of the image at 90 degrees
%----------------------------------------------
%return the two-dimensional Fourier transform of rotated image 
F = fft2(rotatedF,256,256);
figure;
imshow(F);
title('Fast Fourier Transform at 90 degrees')
%----------------------------------------------
% rearranges the fourier transform by shifting the zero-frequency component to the center of the array.
F = fftshift(F);
figure;
imshow(F);
title('Centre of Fast Fourier Transform image at 90 degrees')
%-----------------------------------------------
figure
imshow(abs(F),[0 100]); colormap(jet); colorbar
title('Absolute value for the FFT of the image at 90 degrees')
%----------------------------------------------
%applying logarithm function to the fft image with offset
figure;
imshow(log(1+abs(double(F))),[2,3]); colormap(jet); colorbar
title('Logarithm for the fft of image with offset [2,3] at 90 degrees')
% ---------------------------------------------------------
% Phase angle of image with offset [2,3] and angle 90 degrees
figure;
imshow(angle(double(F)),[-pi,pi]); colormap(jet); colorbar
title('Phase angle for ftt of image with offset [2,3] at 90 degrees')
%---------------------------------------------------------
%;Logarithm of fft image with offset [0,3]
figure;
imshow(log(1+abs(F)),[0,3]); colormap(jet); colorbar
title('Logarithm for fft of image with offset [0,3] at 90 degrees')
%---------------------------------------------------------
%Phase angle of fft image with offset [2,3] at 90 degrees
figure;
imshow(angle(double(F)),[-pi,pi]);colorbar
title('Phase angle for fft of image with offset [-pi,pi] at 90 degrees');
%------------------------------------------------------------
F = rgb2gray(img);
F = imrotate(F,90);
F = fft2(F,256,256);

figure;
imshow(ifft2(F,256,256));
title('inversed fast fourier transform of FFT image at 90 degrees');

