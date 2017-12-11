function [state_curr, pose_curr, vSet] = processFrame_wrapper(I_curr, I_prev, ...
                                                state_prev, KLT_keypointsTracker, ...
                                                KLT_candidateKeypointsTracker, cameraParams)
%PROCESSFRAME_WRAPPER After bootstrap: Estimating remaining camera trajectory
%1. Associate keypoints in the current frame to previously triangulated landmarks.
%2. Based on this, estimate the current camera pose.
%3. Regularly triangulate new landmarks using keypoints not associated to previously triangulated
%landmarks.
%
%   input:
%       camearParams: Object for storing camera parameters
%       vSet: an instance of viewSet
%       I_curr/prev: image of previous and current step (for descriptors)
%       state_prev: previous state, for details see state_curr below
%       KLT_keypointsTracker, KLT_candidateKeypointsTracker:
%           objects of vision.pointTracker to track keypoints and candidate
%           keypoints by KLT algorithm
%
%   output:
%       state_curr: struct containing current state of camera
%           state_curr.keypoints: denoted as P in pdf
%           state_curr.landmarks: denoted as X in pdf
%           state_curr.candidate_kp: candidate keypoint, denoted as C in pdf
%           state_curr.first_obs: first observation of track of keypoint, denoted as F in pdf
%           state_curr.pose_first_obs: pose of first observation above, denoted as T in pdf
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


state_curr = struct([]); 
state_curr.keypoints = [];
state_curr.landmarks = [];
state_curr.candidate_kp = [];
state_curr.first_obs = [];
state_curr.pose_first_obs = [];

pose_curr=1;

end

