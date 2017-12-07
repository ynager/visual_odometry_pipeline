% Parameters File

% dataset (0: KITTI, 1: Malaga, 2: parking)
ds = 0;

% bootstrap images
bootstrap.images = [1,3]; 
bootstrap.det_method = 'harris';

    % harris
    harris.min_quality = 0.2;
    harris.num_points = 100; 
    

ransac.numTrials = 2000;
ransac.distanceThreshold = 0.008; 
ransac.confidence = 99.9; 