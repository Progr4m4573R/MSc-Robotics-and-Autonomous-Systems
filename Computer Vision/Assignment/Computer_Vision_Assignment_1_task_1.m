%Read all images in the folder
%https://uk.mathworks.com/matlabcentral/answers/196072-how-to-read-all-the-images-in-a-folder
myFolder = "C:\Users\Computing\Desktop\MSc-Robotics-and-Autonomous-Systems\Computer Vision\Assignment\skin lesion dataset\org data";
myGTFolder = "C:\Users\Computing\Desktop\MSc-Robotics-and-Autonomous-Systems\Computer Vision\Assignment\skin lesion dataset\GT";
if ~isfolder(myFolder)
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

filePatternGT = fullfile(myGTFolder, '*.png');
GTjpegFiles = dir(filePatternGT);
for k = 1:length(GTjpegFiles)
  baseFileName = GTjpegFiles(k).name;
  fullFileName = fullfile(myGTFolder, baseFileName);
  fprintf(1, 'Now reading %s\n', fullFileName);
  imageArray = imread(fullFileName);
  %imshow(imageArray);  % Display image.
  %drawnow; % Force display to update immediately.
end

%similarity = compute_dice_score(jpegFiles(3),GTjpegFiles(3));
len = length(jpegFiles);
similarity_array=zeros();
for i = 1: len
    similarity = compute_dice_score(jpegFiles(i),GTjpegFiles(i));
    similarity_array = [similarity_array, similarity];
end
mean_score = mean(similarity_array);
