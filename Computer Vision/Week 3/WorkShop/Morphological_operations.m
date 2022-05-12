%https://homepages.inf.ed.ac.uk/rbf/HIPR2/erode.htm
fingerprint = imread('workshop3/noisy-fingerprint.tif');
mushroom_2 = imread('workshop3/mushroom img2.png');
coins = imread('workshop3/coins.gif');

intfp = uint8(mushroom_2);
imshow(coins);
r = 5;
se = strel('ball',5,5);

dilatedI = imdilate(intfp,se);
imshowpair(intfp,dilatedI,'montage')


%erosion
eroded = imerode(mushroom_2,se);
figure
imshow(eroded)
title('eroded mushroom')