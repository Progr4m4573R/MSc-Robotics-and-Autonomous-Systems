clear; close all; clc
%------Task 2 Part 1 select features for radius and direction----------

%https://uk.mathworks.com/help/images/create-image-histogram.html
imgpia = imread('ImgPIA.jpg');
Fourier_Analysis(imgpia)

%-------------Task 2 Part 2 %co-currence matrix----------------------

grayscale = rgb2gray(imgpia);
%https://uk.mathworks.com/help/images/ref/imhist.html
hist = imhist(grayscale);
title('imgpia histogram');
%----------------------------Extract features------------
%https://uk.mathworks.com/matlabcentral/fileexchange/17537-histogram-features-of-a-gray-level-image
%GLCM
features = chip_histogram_features(grayscale);
y = features
x = categorical({'Mean','Variance','Skewness','Kurtosis', 'Energy', 'Enthropy'});
x = reordercats(x,{'Mean','Variance','Skewness','Kurtosis', 'Energy', 'Enthropy'});
subplot(2,2,4)
graph0 = bar(x,y);
title('Features of Gray level co-occurence matrix')
%------------------bit depth of image at 4------------------
bit_depth = 4;
mask = ones(size(grayscale(:,:,1)));
[SRE,LRE,GLN,RP,RLN,LGRE,HGRE] = glrlm(grayscale,bit_depth,mask);
y = [SRE LRE GLN RP RLN LGRE HGRE];
x = categorical({'SRE','LRE','GLN','RP', 'RLN', 'LGRE', 'HGRE'});
x = reordercats(x,{'SRE','LRE','GLN','RP', 'RLN', 'LGRE', 'HGRE'});
subplot(2,2,1)
graph = bar(x,y);
title('Quantise value of 4')
%------------------bit depth of image at 4------------------

%------------------bit depth of image at 6------------------
bit_depth = 6;
mask = ones(size(grayscale(:,:,1)));
[SRE,LRE,GLN,RP,RLN,LGRE,HGRE] = glrlm(grayscale,bit_depth,mask);
y = [SRE LRE GLN RP RLN LGRE HGRE];
x = categorical({'SRE','LRE','GLN','RP', 'RLN', 'LGRE', 'HGRE'});
x = reordercats(x,{'SRE','LRE','GLN','RP', 'RLN', 'LGRE', 'HGRE'});
subplot(2,2,2)
graph2 = bar(x,y);
title('Quantise value of 6')
%------------------bit depth of image at 6------------------

%------------------bit depth of image at 8------------------
bit_depth = 8;
mask = ones(size(grayscale(:,:,1)));
[SRE,LRE,GLN,RP,RLN,LGRE,HGRE] = glrlm(grayscale,bit_depth,mask);
y = [SRE LRE GLN RP RLN LGRE HGRE];
x = categorical({'SRE','LRE','GLN','RP', 'RLN', 'LGRE', 'HGRE'});
x = reordercats(x,{'SRE','LRE','GLN','RP', 'RLN', 'LGRE', 'HGRE'});
subplot(2,2,3)
graph3 = bar(x,y);
title('Quantise value of 8')
%------------------bit depth of image at 8------------------