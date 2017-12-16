function [currState, globalData, viewId] = bootstrap_wrapper(cameraParams, globalData)
%BOOTSTRAP Estimating the pose of the second view relative to the first view.
%Estimate the pose of the second view by estimating the essential matrix and
%decomposing it into camera location and orientation. Triangulation of
%landmarks.
%
%   input:
%       camearParams: Object for storing camera parameters
%           -> (nbr_pts_in_ptcloud x 3) Matrix: [x y z;...]
%       globalData: Object containing actual and estimated viewSets and
%       xyzPoints
%
%   output:
%       globalData: updated globalData
%       currState: struct containing first state of camera
%           currState.keypoints:
%               denoted as P in pdf -> (nbr_kp x 2) Matrix: [u_hor v_vert;...]
%           currState.landmarks:
%               denoted as X in pdf -> (nbr_lm x 3) Matrix: [x y z;...]
%           currState.candidate_kp:
%               candidate keypoint, denoted as C in pdf
%               -> (nbr_ckp x 2) Matrix: [u_hor v_vert;...]
%           currState.first_obs:
%               first observation of track of keypoint, denoted as F in pdf
%               -> (nbr_ckp x 2) Matrix: [u_hor v_vert;...]
%           currState.pose_first_obs: pose of first observation above, denoted as T in pdf
%               -> (nbr_ckp x 12) Matrix: [orientation(:)'loc(:)';...]

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

% load undistorted images
I_0 = loadImage(ds,bootstrap.images(1), cameraParams);
I_1 = loadImage(ds,bootstrap.images(2), cameraParams);

% Detect feature points
switch bootstrap.det_method
    case 'harris'
        points_0 = detectHarrisFeatures(I_0, 'MinQuality', harris.min_quality); %detect
        points_0 = selectUniform(points_0, harris.num_points, size(I_0));       %select uniformly
        
        points_1 = detectHarrisFeatures(I_1, 'MinQuality', harris.min_quality); %detect
        points_1 = selectUniform(points_1, harris.num_points, size(I_1));       %select uniformly
        
    case 'fast'
        points_0 = detectFASTFeatures(I_0, 'MinQuality', fast.min_quality);
        points_0 = selectUniform(points_0, fast.num_points, size(I_0));
        
        points_1 = detectFASTFeatures(I_1, 'MinQuality', fast.min_quality);
        points_1 = selectUniform(points_1, fast.num_points, size(I_1));
        
    otherwise
        disp('given bootstrap.det_method not yet implemented')
end

% USE KLT
if(bootstrap.use_KLT)  
    KLT = vision.PointTracker('NumPyramidLevels', klt.NumPyramidLevels, ...
                              'MaxBidirectionalError', klt.MaxBidirectionalError, ...
                              'BlockSize', klt.BlockSize, ...
                              'MaxIterations', klt.MaxIterations);
    
    initialize(KLT, points_0.Location, I_0);
    [points_klt,kp_validity] = step(KLT,I_1);

    matchedPoints_0 = points_0(kp_validity);
    matchedPoints_1 = points_klt(kp_validity,:);
    
    %convert to cornerPoint object to match template 
    matchedPoints_1 = cornerPoints(matchedPoints_1); 
    indexPairs = ones(matchedPoints_1.Count, 2); 


% USE FEATURE MATCHING
else  
    
    % Extract features at selected points
    switch bootstrap.desc_method
        case 'HOG'
            [descriptors_0, points_0] = extractHOGFeatures(I_0, points_0);
            [descriptors_1, points_1] = extractHOGFeatures(I_1, points_1);
        case 'auto'
            [descriptors_0, points_0] = extractFeatures(I_0, points_0);
            [descriptors_1, points_1] = extractFeatures(I_1, points_1);
        otherwise
            disp('given bootstrap.desc_method not yet implemented')
    end

    % Match features between first and second image.
    indexPairs = matchFeatures(descriptors_0, descriptors_1, 'Unique', true, 'MaxRatio', match.max_ration ,'MatchThreshold', match.match_threshold);

    % estimate pose 
    matchedPoints_0 = points_0(indexPairs(:,1));
    matchedPoints_1 = points_1(indexPairs(:,2));
end

fprintf('\nMatches found: %d\n', length(matchedPoints_0));

% ESTIMATE FUNDAMENTAL MATRIX
for i = 1:bootstrap.ransac_iter
    % this function uses RANSAC and the 8-point algorithm
    [F, inlierIdx] = estimateFundamentalMatrix(matchedPoints_0, matchedPoints_1, ...
                 'Method','RANSAC', 'DistanceThreshold', ransac.distanceThreshold, ... 
                 'Confidence',ransac.confidence, 'NumTrials', ransac.numTrials);
             
    % Make sure we get enough inliers
    ratio = sum(inlierIdx) / numel(inlierIdx); 
    if(ratio > 0.3)
        fprintf('Fraction of inliers: %.2f',ratio);
        break;
    end
end

% get essential matrix (must be transposed for relativeCameraPose)
E = cameraParams.IntrinsicMatrix'*F*cameraParams.IntrinsicMatrix;

% we arent allowed to use directly this function: 
%[E, inlierIdx] = estimateEssentialMatrix(matchedPoints_0, matchedPoints_1, cameraParams); 

% get only matched pairs that are inliers
indexPairs = indexPairs(inlierIdx, :);

% Get the inlier points in each image
inlierPoints_0 = matchedPoints_0(inlierIdx, :);
inlierPoints_1 = matchedPoints_1(inlierIdx, :);

% Compute the camera pose from the fundamental matrix and disambiguate
% invalid configurations using inlierPoints
[orient, loc, validPointFraction] = ...
        relativeCameraPose(E', cameraParams, inlierPoints_0, inlierPoints_1);
fprintf('\nEstimated Location: x=%.2f  y=%.2f  z=%.2f',loc(:));

if(validPointFraction < 0.9) 
    fprintf('\nSmall fraction of valid points when running relativeCameraPose. Essential Matrix might be bad.\n'); 
end

%% Triangulate to get 3D points

% get rotation matrix and translation vector from pose orientation and location
R = orient; 
t = -loc'; 

% calculate camera matrices
M1 = cameraParams.IntrinsicMatrix * eye(3,4); 
M2 = cameraParams.IntrinsicMatrix * [R, t];

% triangulate
xyzPoints = triangulate(inlierPoints_0, inlierPoints_1, M1', M2'); 

%% Generate initial state
% Get unmatched candidate keypoints in second frame wich are all
% elements in points_1 not contained in indexPairs(:,2)

candidate_kp_ind = not(isClose(points_klt,inlierPoints_1.Location,is_close.delta));
% candidate_kp_ind = not(ismember(points_klt,inlierPoints_1.Location,'rows'));
candidate_kp = points_klt(candidate_kp_ind,:);
% candidate_kp_ind = setdiff(1:length(points_klt),indexPairs(:,2));
% candidate_kp = points_klt(candidate_kp_ind,:);

currState.keypoints = inlierPoints_1.Location; 

currState.landmarks = xyzPoints; 
currState.candidate_kp = candidate_kp; 
currState.first_obs = candidate_kp;
currState.pose_first_obs = repmat([orient(:)', loc(:)'], [length(candidate_kp),1]);


%% populate viewsets
viewId = 1; 
globalData.vSet = addView(globalData.vSet, viewId, 'Orientation', eye(3), 'Location', [0 0 0], 'Points', inlierPoints_0);

viewId = 2;
globalData.vSet = addView(globalData.vSet, viewId, 'Orientation', orient, 'Location', loc, 'Points', inlierPoints_1);

% Store the point matches between the previous and the current views.
% globalData.vSet = addConnection(globalData.vSet, viewId-1, viewId, 'Matches', indexPairs);

%% update landmarks and actualVSet in globalData
globalData.landmarks = xyzPoints; 

% delete skipped views in actualVSet
for i = 2:bootstrap.images(2)-1
    try
        globalData.actualVSet = deleteView(globalData.actualVSet, i);
    catch
    end
    
end

%% Plotting

%Plot bootstrap matches
plotMatches(inlierPoints_0, inlierPoints_1, I_0, I_1);  

end

