%% Setup & Load
%load parameters
clear all; 
run('parameters.m');

%get ground truth, K, and last frame index from database
if ds == 0
    % need to set kitti_path to folder containing "00" and "poses"
    kitti_path = '../datasets/kitti';
    assert(exist('../datasets/kitti', 'dir') ~= 0, 'Kitti dataset not found');
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = '../datasets/malaga';
    assert(exist(malaga_path, 'dir') ~= 0, 'Malaga dataset not found');
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = '../datasets/parking';
    assert(exist(parking_path, 'dir') ~= 0, 'Parking dataset not found');
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%put ground truth info into a viewSet
groundTruth = viewSet;
for i = 1:length(ground_truth)
    groundTruth = addView(groundTruth, i, ...
       'Orientation', eye(3), 'Location', [ground_truth(i,1), ground_truth(i,2), 0]);
end


%create cameraParams object
cameraParams = cameraParameters('IntrinsicMatrix', K);
clear K;

%% 	BOOTSTRAP

% load images
I_0 = loadImage(ds,bootstrap.images(1));
I_1 = loadImage(ds,bootstrap.images(2));

% undistort images if nescessary
I_0 = undistortImage(I_0, cameraParams);
I_1 = undistortImage(I_1, cameraParams);

% Detect feature points
switch bootstrap.det_method
    case 'harris'
        points_0 = detectHarrisFeatures(I_0, 'MinQuality', harris.min_quality); %detect
        points_0 = selectUniform(points_0, harris.num_points, size(I_0));       %select uniformly
        
        points_1 = detectHarrisFeatures(I_1, 'MinQuality', harris.min_quality); %detect
        points_1 = selectUniform(points_1, harris.num_points, size(I_1));       %select uniformly
end

% Extract features at selected points
[descriptors_0, points_0] = extractHOGFeatures(I_0, points_0);
[descriptors_1, points_1] = extractHOGFeatures(I_1, points_1);

% Match features between first and second image.
indexPairs = matchFeatures(descriptors_0, descriptors_1, 'Unique', true);

% Plot
imshow(I_0);
hold on;
plot(points_0); 

% Create a ViewSet object with initial orientation and location
vSet = viewSet; 

viewId = 1; 
vSet = addView(vSet, viewId, 'Points', points_0, 'Orientation', eye(3), 'Location', [0 0 0]);
 

% estimate pose 
matchedPoints_0 = points_0(indexPairs(:,1));
matchedPoints_1 = points_1(indexPairs(:,2));

% estimate Fundamental Matrix
% this function uses RANSAC and the 8-point algorithm
[F, inlierIdx] = estimateFundamentalMatrix(matchedPoints_0, matchedPoints_1, ...
                 'Method','RANSAC', 'DistanceThreshold', ransac.distanceThreshold, ... 
                 'Confidence',ransac.confidence, 'NumTrials', ransac.numTrials);
             
E = cameraParams.IntrinsicMatrix'*F*cameraParams.IntrinsicMatrix; 

% get only matched pairs that are inliers
indexPairs = indexPairs(inlierIdx, :);

% Get the inlier points in each image
inlierPoints_0 = matchedPoints_0(inlierIdx, :);
inlierPoints_1 = matchedPoints_1(inlierIdx, :);


%Plot bootstrap matches
figure();
imshow(I_0); 
hold on;
plotMatches(matchedPoints_0, matchedPoints_1);  

    
% Compute the camera pose from the fundamental matrix. Use half of the
% points to reduce computation.
[orient, loc, validPointFraction] = ...
        relativeCameraPose(E, cameraParams, inlierPoints_0(1:2:end, :),...
        inlierPoints_1(1:2:end, :));
    
 if(validPointFraction < 0.9) 
     disp('\nSmall fraction of valid points when running relativeCameraPose. Essential Matrix might be bad.\n'); 
 end


% Add the current view to the view set.
viewId = 2;
vSet = addView(vSet, viewId, 'Points', points_1, 'Orientation', orient, ...
    'Location', loc);
% Store the point matches between the previous and the current views.
vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);

%% Setup Camera/Trajectory plot
[camActual, camEstimated, trajActual, trajEstimated] = setupCamTrajectoryPlot(vSet, groundTruth); 


% update Camera/Trajectory plot
updateCamTrajectoryPlot(viewId, vSet.poses, groundTruth.poses, trajEstimated, trajActual, ...
                        camEstimated, camActual)

    