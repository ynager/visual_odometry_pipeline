%******************************************
% Define calibration parameters
checkerboard_squaresize = 0.021;   % [mm]
device = 'iphone6';

%******************************************

% get calibration images
images = imageSet('calibration_images');
imageFileNames = images.ImageLocation;

[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);
worldPoints = generateCheckerboardPoints(boardSize,checkerboard_squaresize);

% get image size
I = readimage(images,1); 
imageSize = [size(I, 1),size(I, 2)];

% estimate camera parameters using all frames
cameraParams = estimateCameraParameters(imagePoints,worldPoints, ...
                                  'ImageSize',imageSize, ... 
                                  'WorldUnits', 'meters');

% save cameraParams struct
savename = strcat('cameraParams/cameraParams_', device, '.mat'); 
save(savename, 'cameraParams');
fprintf('\nCalibration Done. Saved cameraParams object in %s\n', savename); 

% show the reprojection errors
showReprojectionErrors(cameraParams);

% show the detected and reprojected points
figure; 
imshow(imageFileNames{1}); 
hold on;
plot(imagePoints(:,1,1), imagePoints(:,2,1),'go');
plot(cameraParams.ReprojectedPoints(:,1,1),cameraParams.ReprojectedPoints(:,2,1),'r+');
legend('Detected Points','Reprojected Points');
hold off;