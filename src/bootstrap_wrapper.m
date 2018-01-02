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
%
%   output:
%       viewId: viewID
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

%% source code

% get parameters
run('parameters.m');

% load undistorted images
I_0 = loadImage(ds,bootstrap.images(1), cameraParams);
I_1 = loadImage(ds,bootstrap.images(2), cameraParams);

% Detect feature points
switch bootstrap.det_method
    case 'harris'
        points_0 = detectHarrisFeatures(I_0, 'MinQuality', bootstrap.harris.min_quality, 'FilterSize', processFrame.harris.filter_size); %detect        
        points_1 = detectHarrisFeatures(I_1, 'MinQuality', bootstrap.harris.min_quality, 'FilterSize', processFrame.harris.filter_size); %detect
        
    case 'fast'
        points_0 = detectFASTFeatures(I_0, 'MinQuality', bootstrap.fast.min_quality);        
        points_1 = detectFASTFeatures(I_1, 'MinQuality', bootstrap.fast.min_quality);
        
    otherwise
        disp('given bootstrap.det_method not yet implemented')
end

% Select features among all detected ones
if bootstrap.select_by_nonMax
    points_0 = selectKeypoints(points_0, bootstrap.select_keypoints.delta, bootstrap.select_keypoints.nbr_pts, bootstrap.select_keypoints.viaMatrix_method);
else
    switch bootstrap.det_method
        case 'harris'
            points_0 = selectUniform(points_0, bootstrap.harris.num_points, size(I_0));       %select uniformly
            points_1 = selectUniform(points_1, bootstrap.harris.num_points, size(I_1));       %select uniformly

        case 'fast'
            points_0 = selectUniform(points_0, bootstrap.fast.num_points, size(I_0));
            points_1 = selectUniform(points_1, bootstrap.fast.num_points, size(I_1));

        otherwise
            disp('given bootstrap.det_method not yet implemented')
    end
end

% USE KLT
if(bootstrap.use_KLT)  
    KLT = vision.PointTracker('NumPyramidLevels', bootstrap.klt.NumPyramidLevels, ...
                              'MaxBidirectionalError', bootstrap.klt.MaxBidirectionalError, ...
                              'BlockSize', bootstrap.klt.BlockSize, ...
                              'MaxIterations', bootstrap.klt.MaxIterations);
    
    initialize(KLT, points_0.Location, I_0);
    [points_klt,kp_validity] = step(KLT,I_1);

    matchedPoints_0 = points_0(kp_validity);
    matchedPoints_1 = points_klt(kp_validity,:);
    
    %convert to cornerPoint object to match template 
    matchedPoints_1 = cornerPoints(matchedPoints_1); 


% USE FEATURE MATCHING
else  
    points_1 = selectKeypoints(points_1,bootstrap.select_keypoints.delta, bootstrap.select_keypoints.nbr_pts, bootstrap.select_keypoints.viaMatrix_method);
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
    indexPairs = matchFeatures(descriptors_0, descriptors_1, 'Unique', true, 'MaxRatio', bootstrap.match.max_ration ,'MatchThreshold', bootstrap.match.match_threshold);

    % estimate pose 
    matchedPoints_0 = points_0(indexPairs(:,1));
    matchedPoints_1 = points_1(indexPairs(:,2));
end
plotMatches(matchedPoints_0, matchedPoints_1, I_0, I_1); 
fprintf('\nFeature Matches found: %d\n', length(matchedPoints_0));

% BOOTSTRAP LOOP*************************************************
for bootstrap_ctr = 1:bootstrap.loop.numTrials
    
    % ESTIMATE FUNDAMENTAL MATRIX
    for i = 1:bootstrap.eFm.numTrials
        % this function uses RANSAC and the 8-point algorithm
    	[F,inlierIdx]=estimateFundamental_RANSAC(matchedPoints_0, matchedPoints_1,bootstrap.eFm.ransac.distanceThreshold,bootstrap.eFm.ransac.numTrials);

        % Make sure we get enough inliers
        ratio = sum(inlierIdx) / numel(inlierIdx); 
        if(ratio > bootstrap.eFm.ransac.inlierRatio)
            fprintf('Fraction of inliers for F: %.2f',ratio);
            break;
        elseif i==bootstrap.eFm.numTrials
            fprintf('\nFraction of inliers for F: %.2f\n',ratio);
            disp('Max iterations in estimateFundamentalMatrix trials reached, bad F is likely');
        end
    end

    % get essential matrix (must be transposed for relativeCameraPose)
    E = cameraParams.IntrinsicMatrix'*F*cameraParams.IntrinsicMatrix;

    % Get the inlier points in each image
    inlierPoints_0 = matchedPoints_0(inlierIdx, :);
    inlierPoints_1 = matchedPoints_1(inlierIdx, :);

    % Obtain extrinsic parameters (R,t) from E
    [Rots,u3] = decomposeEssentialMatrix(E);

    % Disambiguate among the four possible configurations
    p0 = [inlierPoints_0.Location ones(length(inlierPoints_0.Location),1)]';
    p1 = [inlierPoints_1.Location ones(length(inlierPoints_1.Location),1)]';
    K = cameraParams.IntrinsicMatrix;
    [orient_C_W,loc_C_W] = disambiguateRelativePose(Rots,u3,p0,p1,K,K);
    orient = orient_C_W';
    loc = -orient*loc_C_W;

    fprintf('\nEstimated Location: x=%.2f  y=%.2f  z=%.2f',loc(:));
    
    %TODO put in a safety measurement? like count pts in front of cam?

    %% Triangulate to get 3D points

    % get rotation matrix and translation vector from pose orientation and location    
    % swap frames, from WC to CW
    R = orient';
    t = -R*loc;

    % calculate camera matrices
    M1 = cameraParams.IntrinsicMatrix * eye(3,4); 
    M2 = cameraParams.IntrinsicMatrix * [R, t];

    % triangulate
    [xyzPoints, reprojectionErrors] = triangulate(inlierPoints_0, inlierPoints_1, M1', M2');
    
    % filter
    [xyzPoints, ind_filt,~,ratio] =  ...
        getFilteredLandmarks(xyzPoints, inlierPoints_1.Location, reprojectionErrors, orient, loc,  bootstrap.triang, cameraParams);

    inlierPoints_0 = inlierPoints_0(ind_filt);
    inlierPoints_1 = inlierPoints_1(ind_filt);

    if ratio>bootstrap.triang.min_landmark_ratio
        fprintf('\nlandmarkfilter reached a ratio of %.2f. \n bootstrap was most likely successful',ratio); 
        break;
    else
        fprintf('\ntoo many rejected landmarks, start bootstrap again, ratio: %.2f',ratio); 
    end

end%*******************************************************


%% Generate initial state
% Get unmatched candidate keypoints in second frame wich are all
% elements in points_1 not contained in indexPairs(:,2)

% remove negative points
pos_rows = and(points_klt(:,1) > 0, points_klt(:,2) > 0);
points_klt = points_klt(pos_rows,:); 

% no candidates close to existing ones
candidate_kp_ind = not(isClose(points_klt,inlierPoints_1.Location,bootstrap.is_close.delta));
candidate_kp = points_klt(candidate_kp_ind,:);

%TODO make this proper
%do not use outliers as candidates
%maybe: use filtered landmarks due to number as candidates, handle case if
%none
candidate_kp = candidate_kp(1,:);

%update current state
currState.keypoints = inlierPoints_1.Location;
currState.landmarks = xyzPoints; 
currState.candidate_kp = candidate_kp; 
currState.first_obs = candidate_kp;
currState.pose_first_obs = repmat([orient(:)', loc(:)'], [length(candidate_kp),1]);


%% populate viewsets
viewId = 1; 
globalData.vSet = addView(globalData.vSet, viewId, 'Orientation', eye(3), 'Location', [0 0 0], 'Points', inlierPoints_0);

viewId = 2;
globalData.vSet = addView(globalData.vSet, viewId, 'Orientation', orient, 'Location', loc', 'Points', inlierPoints_1);

% Store the point matches between the previous and the current views.
% globalData.vSet = addConnection(globalData.vSet, viewId-1, viewId, 'Matches', indexPairs);

%% update landmarks and actualVSet in globalData
globalData.landmarks = xyzPoints; 

%% Plotting

%Plot bootstrap matches
plotMatches(inlierPoints_0, inlierPoints_1, I_0, I_1);  

end

