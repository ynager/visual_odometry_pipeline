%% Setup & Load
%load parameters
clear all; 
close all;
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

%create global data object
globalData.vSet = viewSet;              % viewSet with estimated data
globalData.actualVSet = viewSet;        % viewSet with ground truth
globalData.landmarks = [];              % 3D pointcloud (Nx3 matrix)


%put ground truth info into a realVSet
for i = 1:length(ground_truth)
    globalData.actualVSet = addView(globalData.actualVSet, i, ...
       'Orientation', eye(3), 'Location', [ground_truth(i,1), 0, ground_truth(i,2)]);
end


%create cameraParams object
cameraParams = cameraParameters('IntrinsicMatrix', K);
clear K;

%% 	BOOTSTRAP
close all;

globalData.vSet = viewSet;  %erase if anything in there already

% run bootstrap: Estimating the pose of the second view relative to the first view
[currState, globalData,viewId] = bootstrap_wrapper(cameraParams, globalData);

% apply scale factor to match to ground truth
globalData = applyScaleFactor(globalData); 

%% Setup Camera/Trajectory plot
plotHandles = setupCamTrajectoryPlot(globalData); 

% update Camera/Trajectory plot
I_1 = loadImage(ds,bootstrap.images(2), cameraParams);
updateCamTrajectoryPlot(viewId, globalData,I_1, plotHandles); 

%% Continuous operation

% TODO: fill gap between bootstrap and continuous operation:
% set state_curr to last bootstrap step, I_curr to last bootstrap image
% etc.

% initialize Kanade-Lucas-Tomasi (KLT) point tracker for keypoints and
% candidate keypoints
% TODO: set backtracking in KLT flag true!!
KLT_keypointsTracker = vision.PointTracker('NumPyramidLevels', klt.NumPyramidLevels, ...
                                'MaxBidirectionalError', klt.MaxBidirectionalError, ...
                                'BlockSize', klt.BlockSize, ...
                                'MaxIterations', klt.MaxIterations);
                            
KLT_candidateKeypointsTracker = vision.PointTracker('NumPyramidLevels', klt.NumPyramidLevels, ...
                                'MaxBidirectionalError', klt.MaxBidirectionalError, ...
                                'BlockSize', klt.BlockSize, ...
                                'MaxIterations', klt.MaxIterations);                            

% initialize Tracker
I_curr = loadImage(ds, bootstrap.images(end), cameraParams);
initialize(KLT_keypointsTracker, currState.keypoints, I_curr);
initialize(KLT_candidateKeypointsTracker, currState.candidate_kp, I_curr);
                            
% Estimate remaining camera trajectory
range = (bootstrap.images(2)+1):last_frame;
for i = range
    
    % update to current frame
    prevState = currState;
    I_curr = loadImage(ds,i, cameraParameters);
    
    % get current state (containing all state info) and current pose
    [currState, currRT, globalData] = processFrame_wrapper(I_curr, prevState, ...
                                                   KLT_keypointsTracker, ...
                                                   KLT_candidateKeypointsTracker, ...
                                                   cameraParams, globalData);
    
    % Apply scale factor
    globalData = applyScaleFactor(globalData); 
                                               
    % TODO: update globaldata.viewSet with pose and i
    viewId = i - bootstrap.images(2) + 2; 
    globalData.vSet = addView(globalData.vSet, viewId, 'Orientation', currRT(:,1:3), 'Location', currRT(:,4)', 'Points', currState.keypoints);

    updateCamTrajectoryPlot(viewId, globalData,I_curr, plotHandles); 
    fprintf('\nnum landmarks: %d', length(globalData.landmarks)); 
    
    pause(0.5);
end
    
%% Questions

% keypoints at end of step: still containing the outlier keypoints in p3p?
% i.e. only discard keypoints when failed to track?

% should we implement a delta_loc max threshold? (or orientation
% threshold?)

%% TODO general

% have a look at alpha
% proper printouts -> what causes failure?
               