function [currState, currRT, globalData] = processFrame_wrapper(I_curr, ...
                                                      prevState, KLT_keypointsTracker, ...
                                                      KLT_candidateKeypointsTracker, ...
                                                      cameraParams, globalData)
%PROCESSFRAME_WRAPPER After bootstrap: Estimating remaining camera trajectory
%1. Associate keypoints in the current frame to previously triangulated landmarks.
%2. Based on this, estimate the current camera pose.
%3. Regularly triangulate new landmarks using keypoints not associated to previously triangulated
%landmarks.
%
%   input:
%
%       camearParams: Object for storing camera parameters
%
%       I_curr/prev: image of previous and current step (for descriptors)
%
%       prevState: previous state, for details see currState below
%
%       KLT_keypointsTracker, KLT_candidateKeypointsTracker:
%           objects of vision.pointTracker to track keypoints and candidate
%           keypoints by KLT algorithm
%
%       globalData: struct containting the following
%           globalData.vSet = viewSet; viewSet with estimated data
%           globalData.actualVSet = viewSet; viewSet with ground truth
%           globalData.landmarks = []; 3D pointcloud (Nx3 matrix)
%
%   output:
%
%       currState: struct containing current state of camera
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
%
%       globalData: updated from Input with only pointcloud!
%
%       currRT: current pose and location as [R,T] = [3x3  3x1]
%

%% source code

% get parameters
run('parameters.m');

% track keypoints over frame
[tracked_kp,kp_validity] = step(KLT_keypointsTracker,I_curr);
% track candidate keypoints over frame
[tracked_ckp,ckp_validity] = step(KLT_candidateKeypointsTracker,I_curr);
if debug.print_tracking
    fprintf('\nKLT_kpT tracked: %d out of %d kp.',sum(kp_validity),length(kp_validity));
    fprintf('\nKLT_ckpT tracked: %d out of %d ckp.',sum(ckp_validity),length(ckp_validity));
end 

% make sure tracked_kp and tracked_ckp are not redundant
% TODO: check 1. what values if ckp_validity is false?
% TODO: check 2. if ismember can handle bad values from point 1.
ckp_redundant = isClose(tracked_ckp,tracked_kp,processFrame.is_close.delta);
% ckp_redundant = ismember(tracked_ckp,tracked_kp,'rows'); %check ckp in kp
if debug.print_tracking
    fprintf('\nIn Tracking: found %d redundant ckp',sum(ckp_redundant));
end

% for being ckp member, ckp_validity needs to be 1 but ckp_redundant != 1
wrong_validity = and(ckp_validity,ckp_redundant);
ckp_validity(wrong_validity)=0;
currState.candidate_kp = tracked_ckp(ckp_validity,:); %add candidates from last frames
currState.first_obs = prevState.first_obs(ckp_validity,:);
currState.pose_first_obs = prevState.pose_first_obs(ckp_validity,:);

% run RANSAC to find inliers / run P3P to find new pose
% estimateWorldCameraPose -> is p3p algo of vision toolbox
kp_for_p3p = tracked_kp(kp_validity,:);
landmarks_for_p3p = prevState.landmarks(kp_validity,:);

for i = 1:processFrame.localization.numTrials
    
    % run 3D-2D algo
    [ orient, loc, inlierIdx ] = runP3PandRANSAC( kp_for_p3p, landmarks_for_p3p, cameraParams );
    currRT = [orient,loc];
    
    if(isempty(loc) || isempty(orient))
        continue
    end
    
    % check delta location
    T = globalData.vSet.Views.Location{end}' - loc(:);
    if debug.print_p3p
        fprintf('\nDelta loc: %.2f',norm(T));
    end
    if(norm(T) < processFrame.p3p.max_delta_loc) %Check if position delta is too large
        break; 
    elseif i == processFrame.localization.numTrials
        warning('Max iter. i==%d reached, still large position delta produced!',i)
        %TODO: skip frame if still bad localization
        %TODO: not only check loc but also orient
        
    else
        warning('Large position delta produced after %d iterations!',i)
    end
end

if debug.print_p3p
    fprintf('\nTotal matches found in p3p: %d', sum(inlierIdx));  
    fprintf('\nFraction of inliers(p3p) of input kp in p3p: %.2f',sum(inlierIdx)/length(inlierIdx));
    fprintf('\nFraction of inliers(p3p) of tracked kp: %.2f',sum(inlierIdx)/length(kp_validity));
end

%non-linear optimization of R and T
% represent R and T as twist vector to be useful in lsqnonlin
currRT_twist = HomogMatrix2twist([currRT;[0 0 0 1]]);
% use lonlinear optimization (least squares) to minimize reprojection error
%initialise for lsqnonlin
f=@(RT_twist)rep_e_nonlinopt(RT_twist,double(kp_for_p3p(inlierIdx,:)), double(landmarks_for_p3p(inlierIdx,:)), cameraParams);
%set options for lsqnonlin
options = optimoptions(@lsqnonlin,'Display','iter','FunValCheck','on','MaxIter',400);
% lowerbount = closetocurrRT;
% upperbound = closetocurrRT;
[optRT_twist,squarederrornorm,errors,exitflag] = lsqnonlin(f,currRT_twist,[],[],options);
% twist to optRT
optRT_homo = twist2HomogMatrix(optRT_twist);
optRT = optRT_homo(1:3,1:4);
currRT = optRT; 
orient = currRT(:,1:3); 
loc = currRT(:,4); 

% prepare orient and loc for return
% TODO: check if orient and loc are empty, in that case skip the step
fprintf('\nEstimated Location: x=%.2f  y=%.2f  z=%.2f',loc(:));

% add used keypoints and landmarks to state, discard all others
currState.keypoints = kp_for_p3p(inlierIdx,:);
currState.landmarks = landmarks_for_p3p(inlierIdx,:);


% triangulate new landmarks
[currState,globalData] = triangulateAlphaBased(currState, cameraParams, currRT, globalData);

% Look for and add new candidate keypoints if number of candidates below
% threshold
if(length(currState.candidate_kp) < processFrame.max_candidate_keypoints)
    % detect new candidates 
    switch processFrame.det_method
        case 'harris'
            new_kp = detectHarrisFeatures(I_curr, 'MinQuality', processFrame.harris.min_quality_process, 'FilterSize', processFrame.harris.filter_size);
    %         if harris.selectUniform
    %             new_kp = selectUniform(new_kp, harris.num_points_process, size(I_curr));       %select uniformly
    %             if debug.print_det_method
    %                 fprintf('\nNew features found: %d',size(new_kp,1));
    %             end
    %         end
        otherwise
            warning('given processFrame.det_method not yet implemented in processFrame')
    end

    %%%%%%%%%test: first isClose, then selectUniform%%%%%%%%%%%%
    new_kp_valid = not(isClose(new_kp.Location,[currState.candidate_kp;currState.keypoints],processFrame.select_keypoints.delta));
    new_kp = new_kp(new_kp_valid);
    if processFrame.select_by_nonMax
        new_kp = selectKeypoints(new_kp, processFrame.select_keypoints.delta, processFrame.select_keypoints.nbr_pts, processFrame.select_keypoints.viaMatrix_method);
    else
        new_kp = selectUniform(new_kp, processFrame.harris.num_points, size(I_curr));
    end

    if debug.print_new_features
        fprintf('\nFound %d new features, %d were isClose, %d selected uniform.',length(new_kp_valid), length(new_kp_valid)-sum(new_kp_valid), size(new_kp.Location,1));
    end

    currState.candidate_kp = [currState.candidate_kp; new_kp.Location];
    currState.first_obs = [currState.first_obs; new_kp.Location];
    currState.pose_first_obs = [currState.pose_first_obs;...
        repmat([orient(:)', loc(:)'], [length(new_kp.Location),1])];

end
% finally update KLT_keypointsTracker and KLT_candidateKeypointsTracker
setPoints(KLT_keypointsTracker,currState.keypoints);
setPoints(KLT_candidateKeypointsTracker,currState.candidate_kp);

% printout
fprintf('\nEnd of step. Numbers of keypoints: %d',length(currState.keypoints));


%% Fill up debut plotting data
% get p3p outlier keypoints and landmarks
globalData.debug.p3p_outlier_keypoints = kp_for_p3p(~inlierIdx,:); 
globalData.debug.p3p_outlier_landmarks = landmarks_for_p3p(~inlierIdx,:); 


end

