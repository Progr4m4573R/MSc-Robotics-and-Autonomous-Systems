function output = make_co_occurence_matrix(x)
imshow(x)
f = rgb2gray(x);
figure;
%convert image to grey
imshow(f);
%Modify the offset to change estimationof the co-occurence matrix
%for other angles

D = 1;

glcm_0_s = graycomatrix(f, 'offset', [0 D],'NumLevels', 256, 'Symmetric', true);
figure;
imshow(glcm_0_s);
glcm_0_ns = graycomatrix(f, 'offset', [0 D], 'NumLevels', 256,'Symmetric', false);
figure;
imshow(glcm_0_ns);

stats_0_s = graycoprops(glcm_0_s,{'contrast','correlation','energy','homogeneity'});

output = stats_0_s;
end