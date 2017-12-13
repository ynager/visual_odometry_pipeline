function [currState, currPose, vSet] = processFrame_wrapper(I_curr, I_prev, ...
                                                prevState, KLT_keypointsTracker, ...
                                                KLT_candidateKeypointsTracker, cameraParams, ...
                                                globalData)
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
%       vSet: an instance of viewSet
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
%       vSet: updated vSet of input
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

% Detect feature points
% TODO: this might not be needed!
switch processFrame.det_method
    case 'harris'
        points_curr = detectHarrisFeatures(I_curr, 'MinQuality', harris.min_quality); %detect
    otherwise
        disp('given processFrame.det_method not yet implemented')
end

% track keypoints over frame
[tracked_kp,kp_validity] = step(KLT_keypointsTracker,I_curr); 
% track candidate keypoints over frame
[tracked_ckp,ckp_validity] = step(KLT_candidateKeypointsTracker,I_curr);

% TODO: make sure tracked_kp and tracked_ckp are not redundant

% TODO: run RANSAC to find inliers / run P3P to find new pose
% estimateWorldCameraPose -> is p3p algo of vision toolbox

% TODO: add candidates to state

% TODO: check nbr of matched keypoints -> if e.g. 20% lost -> do traingulation of
% new landmarks based on candiate keypoints
% (in triangulation: check alpha value threshold)


currState = struct([]); 
currState.keypoints = [];
currState.landmarks = [];
currState.candidate_kp = [];
currState.first_obs = [];
currState.pose_first_obs = [];

currPose=1;

end

