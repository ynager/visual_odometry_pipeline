function [currState, currRT, globalData] = re_bootstrap(I_base, base_orient, base_loc, I_curr, prevState, ...
                                            KLT_keypointsTracker, ...
                                            KLT_candidateKeypointsTracker, ...
                                            cameraParams, globalData)
%RE_BOOTSTRAP is like bootstrap but within processFrame, updates the processFrame constructs as well
% code copied from bootstrap (and process frame)

%% source code

% get parameters
run('parameters.m');

% load undistorted images
I_0 = I_base;
I_1 = I_curr;

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
        if(ratio > processFrame.reboot.eFm.ransac.inlierRatio)
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
    
    %% re-bootstrap part
    % scale up loc by number of skipped frames
    scale_factor = processFrame.reboot.stepsize/abs(bootstrap.images(2)-bootstrap.images(1));
    loc = loc*scale_factor;
    % calculate base to world frame
    loc = base_loc + base_orient*loc;
    orient = base_orient*orient;

    %% Triangulate to get 3D points

    % get rotation matrix and translation vector from pose orientation and location    
    % swap frames, from WC to CW
    R1 = base_orient';
    t1 = -R1*base_loc;
    R2 = orient';
    t2 = -R2*loc;

    % calculate camera matrices
    M1 = cameraParams.IntrinsicMatrix * [R1, t1]; 
    M2 = cameraParams.IntrinsicMatrix * [R2, t2];

    % triangulate
    [xyzPoints, reprojectionErrors] = triangulate(inlierPoints_0, inlierPoints_1, M1', M2');
    
    % filter
    [xyzPoints, ind_filt,~,ratio] =  ...
        getFilteredLandmarks(xyzPoints, inlierPoints_1.Location, reprojectionErrors, orient, loc,  processFrame.reboot.triang.radius_threshold, processFrame.reboot.triang.min_distance_threshold, bootstrap.triang.rep_e_threshold, bootstrap.triang.num_landmarks_bootstrap, cameraParams);

    inlierPoints_0 = inlierPoints_0(ind_filt);
    inlierPoints_1 = inlierPoints_1(ind_filt);

    if ratio>processFrame.reboot.triang.min_landmark_ratio
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
currRT = [orient loc];

% finally update KLT_keypointsTracker and KLT_candidateKeypointsTracker
[~,~] = step(KLT_keypointsTracker,I_curr);
[~,~] = step(KLT_candidateKeypointsTracker,I_curr);
setPoints(KLT_keypointsTracker,currState.keypoints);
setPoints(KLT_candidateKeypointsTracker,currState.candidate_kp);

%% Fill up debug plotting data
% get p3p outlier keypoints and landmarks
globalData.debug.p3p_outlier_keypoints = []; 
globalData.debug.p3p_outlier_landmarks = [];
globalData.debug.ckeypoints_invalid = [];


%% update landmarks and actualVSet in globalData
globalData.landmarks = [globalData.landmarks; xyzPoints]; 

%% Plotting

%Plot bootstrap matches
% plotMatches(inlierPoints_0, inlierPoints_1, I_0, I_1);  

end

