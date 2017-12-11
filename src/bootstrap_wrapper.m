function [vSet, viewId] = bootstrap_wrapper(cameraParams, vSet)
%BOOTSTRAP Estimating the pose of the second view relative to the first view.
%Estimate the pose of the second view by estimating the essential matrix and
%decomposing it into camera location and orientation. Triangulation of
%landmarks.
%
%   input:
%       camearParams: Object for storing camera parameters
%       vSet: an instance of viewSet
%
%   output:
%       vSet: updated vSet of input
%       state_first: struct containing first state of camera
%           state_first.keypoints: denoted as P in pdf
%           state_first.landmarks: denoted as X in pdf
%           state_first.candidate_kp: candidate keypoint, denoted as C in pdf
%           state_first.first_obs: first observation of track of keypoint, denoted as F in pdf
%           state_first.pose_first_obs: pose of first observation above, denoted as T in pdf

% TODOs: 
% -(maybe) use viewSet only for visualization. for Pose, landmarks storage
% etc. use proposed state of project-pdf. (see state_first)
% -based on correspondences between keypoints: triangulate to get landmarks
% -properly save data of second bootstrap image,keyponts,landmarks etc. in
% first instance of struct state_curr (here called state_first)
% -update input and output infos in function header

% NOTES:
% establish keypoint correspondences between bootstrap images can be
% done by either patch matching or KLT. Currently patch matching is
% implemented.

%% source code

% get parameters
run('parameters.m');

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
    otherwise
        disp('given bootstrap.det_method not yet implemented')
end

% Extract features at selected points
switch bootstrap.desc_method
    case 'HOG'
        [descriptors_0, points_0] = extractHOGFeatures(I_0, points_0);
        [descriptors_1, points_1] = extractHOGFeatures(I_1, points_1);
    otherwise
        disp('given bootstrap.desc_method not yet implemented')
end

% Match features between first and second image.
indexPairs = matchFeatures(descriptors_0, descriptors_1, 'Unique', true);

% Plot
imshow(I_0);
hold on;
plot(points_0); 

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


end

