%Prepare image
%https://homepages.inf.ed.ac.uk/rbf/HIPR2/fourier.htm - helpful
%understanding
f = zeros(256,256);
f(43:213,111:146) = 1;

figure;
imshow(f);
title('a 256 by 256 array of 0s')
% Compute Fourier Transform
F = fft2(f,256,256);
figure;
imshow(F);
title('Fourier Transform')
F = fftshift(F); % Center FFT
figure;
imshow(F);
title('Centre the Fast Fourier Transform')
% Measure the minimum and maximum value of the transform amplitude
min(min(abs(F)))
max(max(abs(F)))
figure;
imshow(abs(F),[0 100]); colormap(jet); colorbar
title('Amplitude')
figure;
imshow(log(1+abs(F)),[0,3]); colormap(jet); colorbar
title('Logarithm of Amplitude')
% Look at the phases
figure;
imshow(angle(F),[-pi,pi]); colormap(jet); colorbar
title('Phases')

%inverse the Fourier Fast Transform
I = ifft2(F, 256, 256);
figure;
imshow(I);
title('Inverse of Fast Fourier Transform')


I = fftshift(I); % Center FFT
figure;
imshow(I);
title('Centre the 3RD Fourier Transform')

figure;
imshow(log(1+abs(I)),[0,3]); colormap(jet); colorbar
title('Logarithm of 3RD Amplitude')