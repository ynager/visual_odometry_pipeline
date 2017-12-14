% Parameters File

% dataset (0: KITTI, 1: Malaga, 2: parking)
ds = 2;

% bootstrap images
bootstrap.images = [1,5]; 
bootstrap.det_method = 'harris';
bootstrap.desc_method = 'HOG';

bootstrap.use_KLT = true; 

    % harris
    harris.min_quality = 0.001; % 0.001 init
    harris.num_points = 5000; % 10000 init
    
    % fast
    fast.min_quality = 0.01;
    fast.num_points = 20000; 
    

ransac.numTrials = 10000;
ransac.distanceThreshold = 0.01; 
ransac.confidence = 99.9; 

%feature matching
match.max_ration = 0.6;    %(0,1)
match.match_threshold = 10; %(0,100)

% continuous operation
processFrame.det_method = bootstrap.det_method;
%processFrame.desc_method = bootstrap.desc_method;

    % KLT point Tracker
    % https://ch.mathworks.com/help/vision/ref/vision.pointtracker-system-object.html
    klt.NumPyramidLevels = 3; % TODO: nbr was mentioned in lecture
    klt.MaxBidirectionalError = 2; % if inf, is not calculated
    klt.BlockSize = [31 31];
    klt.MaxIterations = 30;
    
    p3p_ransac.num_iteration = 1000;
    p3p_ransac.pixel_tolerance = 2; % 2 init
    p3p_ransac.min_inlier = 20;
    
    % harris
    harris.selectUniform = true;
    
    % triangulation
    alpha_threshold = deg2rad(10); % 20 init