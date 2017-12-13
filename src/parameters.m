% Parameters File

% dataset (0: KITTI, 1: Malaga, 2: parking)
ds = 2;

% bootstrap images
bootstrap.images = [15,18]; 
bootstrap.det_method = 'harris';
bootstrap.desc_method = 'HOG';

    % harris
    harris.min_quality = 0.05;
    harris.num_points = 10000; 
    
    % fast
    fast.min_quality = 0.01;
    fast.num_points = 1000; 
    

ransac.numTrials = 10000;
ransac.distanceThreshold = 0.01; 
ransac.confidence = 99.9; 

%feature matching
match.max_ration = 0.6;    %(0,1)
match.match_threshold = 10; %(0,100)

% continuous operation
%processFrame.det_method = bootstrap.det_method;
%processFrame.desc_method = bootstrap.desc_method;

    % KLT point Tracker
    % https://ch.mathworks.com/help/vision/ref/vision.pointtracker-system-object.html
    klt.NumPyramidLevels = 3; % TODO: nbr was mentioned in lecture
    klt.MaxBidirectionalError = 2; % if inf, is not calculated
    klt.BlockSize = [31 31];
    klt.MaxIterations = 30;