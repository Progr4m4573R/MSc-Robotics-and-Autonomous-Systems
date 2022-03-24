%https://homepages.inf.ed.ac.uk/rbf/HIPR2/erode.htm
fingerprint = imread('noisy-fingerprint.tif');
mushroom_2 = imread('mushroom img2.png');
coins = imread('coins.gif');

intfp = uint8(fingerprint);
imshow(coins);
r = 5;
se = strel('ball',5,5);

dilatedI = imdilate(intfp,se);
imshowpair(intfp,dilatedI,'montage')

