function similarity = compute_dice_score(x,y)
%classify each image's colours in RGB Colour space using k-means clustering
%https://uk.mathworks.com/help/images/color-based-segmentation-using-k-means-clustering.html
%Step 1;
    im  = imread(x.name);
    gti = imread(y.name);
    %figure
    %imshow(im(i));
    %title("Baseline image")
    %Step 2 
    numColors = 3;
    L = imsegkmeans(im,numColors);
    B = labeloverlay(im,L);
    % figure
    % imshow(B)
    % title("Labeled IMage RGB")
    %Step 3 Convert Image from RGB color space to L*a*b
    lab_im = rgb2lab(im);
    %Step 4 classify colours in a*b* space using k-means clustering
    ab = lab_im(:,:,2:3);
    ab = im2single(ab);
    pixel_labels = imsegkmeans(ab,numColors,"NumAttempts",3);

    %Display labelled image as overlay on original image
    B2 = labeloverlay(im,pixel_labels);
    % figure
    % imshow(B2);
    % title("labelled Image a*b*");

    %Step 5 Create Images that Segment H&E Image by Colour
    %First mask is the outer layer of the lesion
    mask1 = pixel_labels==1;
    cluster1 = im.*uint8(mask1);
    % figure
    % imshow(cluster1);
    % title("Objects in Cluster 1");
    %Objects in second cluster, the skin
    mask2 = pixel_labels ==2;
    cluster2 = im.*uint8(mask2);
    %figure
    %imshow(cluster2);
    %title("Objects in Cluster 2");
    %Objects in third cluster, inner parts of the lesion
    mask3 = pixel_labels == 3;
    cluster3 = im.*uint8(mask3);
    % figure
    % imshow(cluster3);
    % title("Objects in Cluster 3");
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
    % figure
    % imshow(lesion_nuclei);
    % title("Lesion Nuclei");

    %Conver to black and white
    %https://uk.mathworks.com/help/images/ref/imbinarize.html?s_tid=doc_ta

    %convert cluster 2 to a binary image to draw a boundary around leison
    se = strel('ball',5,5);

    eroded = imerode(cluster2,se);

    grey_cluster = rgb2gray(eroded);
    BW = imbinarize(grey_cluster);
    % figure
    % imshowpair(lesion_nuclei,BW, 'montage')
    % title("lesion before and after conversion")
    %figure
    result = edge(BW,'log');
    dim = size(BW);
    col = round(dim(2)/2)-90;
    row = min(find(result(:,col)));
    boundary = bwtraceboundary(result,[row,col],'N');
    %imshow(result)
    hold on;
    plot(boundary(:,2),boundary(:,1),'g','LineWidth',3);
    final = imfill(result,'holes');
    %erode

    %imshow(final)
    %title("eroded output")

    %binarize the ground truth to compare

    y = imbinarize(gti);
    %https://uk.mathworks.com/help/images/ref/dice.html
    similarity = dice(final,y);
end