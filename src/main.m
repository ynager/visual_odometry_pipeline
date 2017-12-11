%% Setup & Load
%load parameters
clear all; 
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

%put ground truth info into a viewSet
groundTruth = viewSet;
for i = 1:length(ground_truth)
    groundTruth = addView(groundTruth, i, ...
       'Orientation', eye(3), 'Location', [ground_truth(i,1), ground_truth(i,2), 0]);
end


%create cameraParams object
cameraParams = cameraParameters('IntrinsicMatrix', K);
clear K;

%% 	BOOTSTRAP

% Create a ViewSet object with initial orientation and location
vSet = viewSet; 

% run bootstrap: Estimating the pose of the second view relative to the first view
[vSet,viewId] = bootstrap_wrapper(cameraParams, vSet);

%% Setup Camera/Trajectory plot
[camActual, camEstimated, trajActual, trajEstimated] = setupCamTrajectoryPlot(vSet, groundTruth); 


% update Camera/Trajectory plot
updateCamTrajectoryPlot(viewId, vSet.poses, groundTruth.poses, trajEstimated, trajActual, ...
                        camEstimated, camActual)


%% Continuous operation

% TODO: fill gap between bootstrap and continuous operation:
% set state_curr to last bootstrap step, I_curr to last bootstrap image
% etc.

% initialize Kanade-Lucas-Tomasi (KLT) point tracker for keypoints and
% candidate keypoints
KLT_keypointsTracker = vision.PointTracker('NumPyramidLevels', klt.NumPyramidLevels, ...
                                'MaxBidirectionalError', klt.MaxBidirectionalError, ...
                                'BlockSize', klt.BlockSize, ...
                                'MaxIterations', klt.MaxIterations);
                            
KLT_candidateKeypointsTracker = vision.PointTracker('NumPyramidLevels', klt.NumPyramidLevels, ...
                                'MaxBidirectionalError', klt.MaxBidirectionalError, ...
                                'BlockSize', klt.BlockSize, ...
                                'MaxIterations', klt.MaxIterations);                            

initialize(KLT_keypointsTracker, I_curr.keypoints, I_curr);
initialize(KLT_candidateKeypointsTracker, I_curr.candidate_kp, I_curr);
                            
% Estimate remaining camera trajectory
range = (bootstrap.images(2)+1):last_frame;
for i = range
    
    % update to current frame
    state_prev = state_curr;
    I_prev = I_curr;
    I_curr = loadImage(ds,i);
    
    % get current state (containing all state info) and current pose
    [state_curr, pose_curr] = processFrame_wrapper(I_curr, I_prev, state_prev, ...
                                                   KLT_keypointsTracker, ...
                                                   KLT_candidateKeypointsTracker, ...
                                                   cameraParams);
    
    % TODO: update vSet with new pose
    
end
    
                    
               