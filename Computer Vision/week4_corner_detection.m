%https://uk.mathworks.com/help/images/ref/corner.html
I = imread('checkERbox-grey.jpg');
C  = corner(I,'Harris'); 

subplot(1,2,1);
imshow(I);
hold on
plot(C(:,1), C(:,2), '*', 'Color', 'c')
title('Maximum Corners = 200')
hold off

corners_max_specified = corner(I,3);
subplot(1,2,2);
imshow(I);
hold on
plot(corners_max_specified(:,1), corners_max_specified(:,2), ...
   '*', 'Color', 'm')
title('Maximum Corners = 3')
hold off

%https://uk.mathworks.com/help/vision/ref/detectharrisfeatures.html
harriscorners = detectHarrisFeatures(I);
imshow(I); hold on;
plot(harriscorners.selectStrongest(50));
title('Harris corners')