%Read all images in the folder
%https://uk.mathworks.com/matlabcentral/answers/196072-how-to-read-all-the-images-in-a-folder
myFolder = 'C:\Users\Computing\Desktop\MSc-Robotics-and-Autonomous-Systems\Computer Vision\Assignment\skin lesion dataset\org data';
if ~isdir(myFolder)
  errorMessage = sprintf('Error: The following folder does not exist:\n%s', myFolder);
  uiwait(warndlg(errorMessage));
  return;
end
filePattern = fullfile(myFolder, '*.jpg');
jpegFiles = dir(filePattern);
for k = 1:length(jpegFiles)
  baseFileName = jpegFiles(k).name;
  fullFileName = fullfile(myFolder, baseFileName);
  fprintf(1, 'Now reading %s\n', fullFileName);
  imageArray = imread(fullFileName);
  %imshow(imageArray);  % Display image.
  %drawnow; % Force display to update immediately.
  
end

%classify each image's colours in RGB Colour space using k-means clustering
%https://uk.mathworks.com/help/images/color-based-segmentation-using-k-means-clustering.html
%Step 1;
im  = imread(jpegFiles(1).name);
imshow(im);
%Step 2 
numColors = 3;
L = imsegkmeans(im,numColors);
B = labeloverlay(im,L);
imshow(B)
title("Labeled IMage RGB")
%Step 3 Convert Image from RGB color space to L*a*b
lab_im = rgb2lab(im);
%Step 4 classify colours in a*b* space using k-means clustering
ab = lab_im(:,:,2:3);
ab = im2single(ab);
pixel_labels = imsegkmeans(ab,numColors,"NumAttempts",3);

%Display labelled image as overlay on original image
B2 = labeloverlay(he,pixel_labels);
imshow(B2);
title("labelled Image a*b*");

%Step 5 Create Images that Segment H&E Image by Colour
%First mask is the outer layer of the lesion
mask1 = pixel_labels==1;
cluster1 = im.*uint8(mask1);
imshow(cluster1);
title("Objects in Cluster 1");
%Objects in second cluster, the skin
mask2 = pixel_labels ==2;
cluster2 = im.*uint8(mask2);
imshow(cluster2);
title("Objects in Cluster 2");
%Objects in third cluster, inner parts of the lesion
mask3 = pixel_labels == 3;
cluster3 = im.*uint8(mask3);
imshow(cluster3);
title("Objects in Cluster 3");
%Step 6 Segment Nuclei
L = lab_im(:,:,1);
L_inner_lesion = L.*double(mask3);
L_inner_lesion = rescale(L_inner_lesion);
idx_light_L_inner_lesion = imbinarize(nonzeros(L_inner_lesion));

%copy the mask of the lesion and remove light lesions from the mask
lesion_idx = find(mask3);
mask_dark_lesion = mask1;
mask_dark_lesion(lesion_idx(idx_light_L_inner_lesion)) = 0;
lesion_nuclei = im.*uint8(mask_dark_lesion);
imshow(lesion_nuclei);
title("Lesion Nuclei");