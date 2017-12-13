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

% load images
I_0 = loadImage(ds,bootstrap.images(1), cameraParams);
I_1 = loadImage(ds,bootstrap.images(2), cameraParams);

% undistort images if nescessary
%I_0 = undistortImage(I_0, cameraParams);
%I_1 = undistortImage(I_1, cameraParams);

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

% Add to vSet
viewId = 1; 
globalData.vSet = addView(globalData.vSet, viewId, 'Points', points_0, 'Orientation', eye(3), 'Location', [0 0 0]);
 
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
plotMatches(matchedPoints_0, matchedPoints_1, I_0, I_1);  

    
% Compute the camera pose from the fundamental matrix.
[orient, loc, validPointFraction] = ...
        relativeCameraPose(E, cameraParams, inlierPoints_0, inlierPoints_1);
    
 if(validPointFraction < 0.9) 
     fprintf('\nSmall fraction of valid points when running relativeCameraPose. Essential Matrix might be bad.\n'); 
 end


% Add the current view to the view set.
viewId = 2;
globalData.vSet = addView(globalData.vSet, viewId, 'Orientation', orient, 'Location', loc);

% Store the point matches between the previous and the current views.
% globalData.vSet = addConnection(globalData.vSet, viewId-1, viewId, 'Matches', indexPairs);

%% Triangulate to get 3D points

% Get 2 possible rotation matrices and a translation vector
[Rots, u3] = decomposeEssentialMatrix(E);

% Disambiguate invalid configurations (makes sure majority of points lie in
% front of camera) 
[R,T] = disambiguateRelativePose(Rots,u3,inlierPoints_0, inlierPoints_1,cameraParams);

% calculate camera matrices
M1 = cameraParams.IntrinsicMatrix * eye(3,4); 
M2 = cameraParams.IntrinsicMatrix * [R, T];

% triangulate
xyzPoints = triangulate(inlierPoints_0, inlierPoints_1, M1', M2'); 

%% Generate initial state
% Get unmatched candidate keypoints in second frame wich are all
% elements in points_1 not contained in indexPairs(:,2)
candidate_kp_ind = setdiff(1:length(points_1.Location),indexPairs(:,2));
candidate_kp = points_1.Location(candidate_kp_ind,:);

currState.keypoints = inlierPoints_1.Location; 
currState.landmarks = xyzPoints; 
currState.candidate_kp = candidate_kp; 
currState.first_obs = candidate_kp;
currState.pose_first_obs = repmat([orient(:)', loc(:)'], [length(candidate_kp),1]);

%% update landmarks and actualVSet in globalData
globalData.landmarks = xyzPoints; 

% delete skipped views in actualVSet
for i = 2:bootstrap.images(2)-1
    try
        globalData.actualVSet = deleteView(globalData.actualVSet, i);
    catch
    end
    
end

end

