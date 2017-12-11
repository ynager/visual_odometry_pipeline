% Parameters File

% dataset (0: KITTI, 1: Malaga, 2: parking)
ds = 2;

% bootstrap images
bootstrap.images = [1,3]; 
bootstrap.det_method = 'harris';
bootstrap.desc_method = 'HOG';

    % harris
    harris.min_quality = 0.2;
    harris.num_points = 100; 
    

ransac.numTrials = 2000;
ransac.distanceThreshold = 0.008; 
ransac.confidence = 99.9; 

% continuous operation
processFrame.det_method = bootstrap.det_method;
processFrame.desc_method = bootstrap.desc_method;

    % KLT point Tracker
    % https://ch.mathworks.com/help/vision/ref/vision.pointtracker-system-object.html
    klt.NumPyramidLevels = 3; % TODO: nbr was mentioned in lecture
    klt.MaxBidirectionalError = 2; % if inf, is not calculated
    klt.BlockSize = [31 31];
    klt.MaxIterations = 30;