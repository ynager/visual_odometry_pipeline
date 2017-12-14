function [currState, currRT, globalData] = processFrame_wrapper(I_curr, I_prev, ...
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

% TODOs: 
% -Use KLT tracking+RANSAC (of exercise 7, for KLT use vision.PointTracker)
% instead of matchFeatures+RANSAC. to match keypoints of new image to
% previous image
% -add new not-machted(tracking failed) keypoints to candidate keypoints
% -Track candidate keyppoints (and keypoints).
% -Estimate current pose by P3P, do this in RANSAC step of above
% -check in every step the portion of used landmarks: compare matched
% keyponts to available keyponts. if e.g. 20% lost: triangulate new
% landmarks based on candidate keyponts.
% PROBLEM: how to find 20%? maybe add nbr_landmark_threshold to state???
% -(maybe) check that new to-be-added candidate keypoints are not redundant
% with other keypoints or candidate keypoints
% -in triangulation: check threshold alpha before using traingulated point.
% -update input and output infos in function header
% -check where lens distortion needs to be considered

%% source code

% get parameters
run('parameters.m');

% track keypoints over frame
[tracked_kp,kp_validity] = step(KLT_keypointsTracker,I_curr); 
% track candidate keypoints over frame
[tracked_ckp,ckp_validity] = step(KLT_candidateKeypointsTracker,I_curr);

% make sure tracked_kp and tracked_ckp are not redundant
% TODO: check 1. what values if ckp_validity is false?
% TODO: check 2. if ismember can handle bad values from point 1.
% TODO: is a isclose needed?
ckp_redundant = ismember(tracked_ckp,tracked_kp,'rows'); %check ckp in kp

% run RANSAC to find inliers / run P3P to find new pose
% estimateWorldCameraPose -> is p3p algo of vision toolbox
landmarks_for_p3p = prevState.landmarks(kp_validity,:);
kp_for_p3p = tracked_kp(kp_validity,:);
% TODO: check if estimateWorldCameraPose is allowed? -> does not use ransac
% TODO: if this fct works here, add maybe parameters
% [orient, loc, inlierIdx] = estimateWorldCameraPose(kp_for_p3p, landmarks_for_p3p, cameraParams,'MaxReprojectionError',10,'Confidence',90);
[ orient, loc, inlierIdx ] = runP3PandRANSAC( kp_for_p3p, landmarks_for_p3p, cameraParams );

% prepare orient and loc for return
% TODO: check if orient and loc are empty, in that case skip the step?
currRT = [orient,loc];

% TODO: add candidates to state
currState.keypoints = kp_for_p3p(inlierIdx,:);
currState.landmarks = landmarks_for_p3p(inlierIdx,:);

% for being ckp member, ckp_validity needs to be 1 but ckp_redundant != 1
% TODO: use and
wrong_validity = and(ckp_validity,ckp_redundant);
ckp_validity(wrong_validity)=0;
currState.candidate_kp = tracked_ckp(ckp_validity,:); %add candidates from last frames
currState.first_obs = prevState.first_obs(ckp_validity,:);
currState.pose_first_obs = prevState.pose_first_obs(ckp_validity,:);

% TODO: check if inlierIdx are below thershold, if yes run triangulation
% with candidate_kp, dont forget to set lvl to new value
% TODO: check if alpha of landmark is above threshold -> triangulate for
% those
[currState] = triangulateAlphaBased(currState);
bearings_curr = getBearingVector( currState.candidate_kp, cameraParams.IntrinsicMatrix );
bearings_curr_W = orient*bearings_curr'
bearings_prev = getBearingVector( currState.first_obs, cameraParams.IntrinsicMatrix );
for i = 1:size(currState.pose_first_obs,1)
    bear = bearings_prev(i,:);
    R = currState.pose_first_obs(i,:);
    R = R(1:9);
    R = reshape(R,3,3);
    bearings_prev_W = R*bear'; %attention change of convention of axes here
    %TODO: test if angle is always small or vectors potentialle can swap
    alpha = atan2(norm(cross(bearings_prev_W,bearings_curr_W(:,i)),dot(bearings_prev_W,bearings_curr_W(:,i))));
    if alpha > triangulaiton.alpha
        %TODO: do triangulation, delete from state in candidate, add to
        %state in keypoint and add landmark to pointcloud...and others?
    end
end
% TODO: after triang, append used candidates to curr states, append landmarks to
% curr landmarks, and delete used candidates from candidate list and first
% obs list

% detect new candidates 
switch processFrame.det_method
    case 'harris'
        new_kp = detectHarrisFeatures(I_curr, 'MinQuality', harris.min_quality);
        if harris.selectUniform
            new_kp = selectUniform(new_kp, harris.num_points, size(I_curr));       %select uniformly
        end
    otherwise
        disp('given processFrame.det_method not yet implemented')
end

% check for redundancy and add new candidates to state and current pose to
% first obs
!!!!!!!!!!!############
new_kp_redundant = ismember(...,'rows'); %check ckp in kp



currState = struct([]); 
currState.keypoints = [];
currState.landmarks = [];
currState.candidate_kp = [];
currState.first_obs = [];
currState.pose_first_obs = [];


end

