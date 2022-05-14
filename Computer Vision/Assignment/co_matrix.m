%https://www.mathworks.com/help/images/fourier-transform.html
function[] =co_matrix(Img_val)

%Task 2 part 2 Feature Calculation:

%CLCM Matrix
[rows, columns, channelscount] = size(Img_val);
if channelscount > 1
    grayImage = rgb2gray(Img_val);
else
    grayImage  = Img_val;
end
integerValue = double(256);
numGLRanges = integerValue;

D = 1;

%GLCM with graycomatrix().
kernel = [0,D;-D,D;-D,0;-D,-D];
glcm = graycomatrix(grayImage, 'NumLevels', numGLRanges,'GrayLimits',[],'offset',kernel);
numOfDir = size(glcm, 3);
for k = 1:numOfDir
    glcm(:,:,k)
end
%create co_occurence matrix
glcm = sum(glcm,3);
rgbImage = ind2rgb(glcm,hot(numGLRanges));
figure(1)

subplot(2,2,1)
imshow(Img_val);
title('Original image')
subplot(2,2,2)
imshow(rgbImage)
title('image for GLCM');

%First order feature for image histogram
%extract first order freatures from co_occurence matrix
[pix_Count, GrayLevels] = imhist(rgbImage);
Total_pix = sum(pix_Count);
GrayLevel_mean = sum(GrayLevels .* (pix_Count / Total_pix));

Feature_variance = 0; 
Feature_skewness = 0;
Feature_kurtosis = 0;
for i = 0:1:length(pix_Count)-1
    Feature_variance = Feature_variance + (i-GrayLevel_mean)^2 * (pix_Count);
    Feature_skewness = Feature_skewness + (i-GrayLevel_mean)^3 * (pix_Count);
    Feature_kurtosis = Feature_kurtosis + (i-GrayLevel_mean)^4 * (pix_Count);
end
Feature_skewness = Feature_skewness * Feature_variance ^-3; 
Feature_kurtosis = Feature_kurtosis * Feature_variance ^-4;
Feature_energy = sum((pix_Count /Total_pix) .^ 2);
pI = pix_Count /Total_pix;
Feature_entropy = -sum(pI(pI~=0).* log(pI(pI~=0)));
First_order_feature = [GrayLevel_mean, Feature_variance,Feature_skewness,Feature_kurtosis,Feature_energy,Feature_entropy];
figure(1)
subplot(2,2,3)
bar(First_order_feature)
title('1st order feature of co_occurence matrix')
%Gray level run matrix
image_I = double(rgb2grat(Img_val));
Image_min=min(image_I);
Image_N = Image_I - Image_min;
Norm_Max_intensity = max(Image_N);
Quantize_T = round(Norm_Max_intensity/16);
[row_M,col_N] = size(Image_N);
Quantized_val = 0;
for i=1:row_M
    for j=1:col_N
        Img_val = Image_N(i,j);
        for B = 1:16
            if (Img_val >Quantized_val) && (Img_val <Quantized_val+Quantize_T)
                Image_N(i,j)=B/16;
                Quantized_val = Quantized_val+Quantize_T;
            end
        end
    end
end
Max_v = max(Image_N);
Min_v = Image_N./Max_v;
nImage_new=round(Min_v*16)+1;
direction_val = 0;
distance_val = 1;

if (direction_val ==1)
    nImage_new = nImage_new';
end
mx_v = max(max(nImage_new));
mn_v = min(min(nImage_new));
gl = (mx_v-mn_v) +1;
[p_row,q_col] = size(nImage_new);
col_N = p_row*q_col;
count_v = 1;
c =1;
col = 1;
gray_rl(mx_v,p_row) = 0;
maximum_count(p_row*q_col) = 0;
mc = 0;
%Computing Gray level run length matrix
for j = 1:p_row
    for k = 1:q_col-distance_val
        mc=mc+1;
        g=nImage_new(j,k);
        f = nImage_new(j,k+distance_val);
        if(g==f) && (g~=0)
            count_v=count_v+1;
            c = count_v;
            col = count_v;
            maximum_count(mc)=count_v;
        else
            gray_rl(g,c) = gray_rl(g,c) +1;col=1;
            count_v=1;
            c=1;
        end
    end
    gray_rl(f,col)=gray_rl(f,col)+1;
    count_v=1;
    c=1;
end
Img_val=(mx_v:mn_v);
row_M=gray_rl(mn_v:mx_v,:);
mf1=row_M';
maxrun=max(max(maximum_count));
S_val=0;
G_val(gl)=0;
R_val(q_col)=0;
for u=1:gl
    for v=1:p_row
        G_val(u)=G_val(u)+row_M(u,v);
        S_val=S_val +row_M(u,v);
    end
end
for u1=1:p_row
    for v1=1:gl
       R_val(u1)=R_val(u1)+mf1(u1,v1);
    end
end
[dim,dim1] = size(G_val);
feature_SRE=0;feature_LRE=0;feature_GLN=0;feature_RLN=0;feature_RP=0;

for h1=1:maxrun
   feature_SRE=feature_SRE+(R_val(h1)/(h1*h1));
   feature_LRE=feature_lRE+(R_val(h1)*(h1*h1));
   feature_RLN=featureRLN+(R_val(h1)*R_val(h1));
   feature_RP=feature_RP+R_val(h1); 
end
fea_SRE1=feature_SRE/s_val;
fea_LRE1=feature_LRE/s_val;
fea_RLN1=feature_RLN/s_val;
fea_RP1=feature_RP/col_N;

for h2=1:g1
    feature_GLN = (feature_GLN+G_val(h2)^2);
end

fea_GLN1=feature_GLN/s_val;
Feature_GLRLM = [fea_SRE1 fea_LRE1 fea_RLN1 fea_RP1 fea_GLN1];
figure(1)
subplot(2,2,4)
bar(Feature_GLRLM)
title('Feature GLRLM')