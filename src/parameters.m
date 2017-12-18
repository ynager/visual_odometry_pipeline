% Parameters File

% dataset (0: KITTI, 1: Malaga, 2: parking)
ds = 2;

% debugging
debug.print_tracking = true; %prints for Tracking in processFrame
debug.print_p3p = true; %prints for p3p in processFrame
debug.print_triangulation = true; % for alpha value based triangulation in processFrame
debug.print_new_landmarks = true; % for nbr of new landmarks in triangulation
debug.print_det_method = true; % for detection of new keypoints
debug.keyboard_interrupt = false; %interruption after each frame

% params for datasets
switch(ds)
    case 2  %pPARKING
        % BOOTSTRAP images
        bootstrap.numTrials = 100; %run bootstrap max numTrial times
        bootstrap.x_interval = [0.99 1.01]; %bootstrap accepted if x value in this interval
        
        bootstrap.images = [1,3]; 
        bootstrap.det_method = 'harris';
        bootstrap.desc_method = 'HOG';

        bootstrap.use_KLT = true; 
        bootstrap.ransac_iter = 1000; %nbr of estimateFundamentalMatrix runs

            % harris
            harris.min_quality = 0.001; % 0.001 init
            harris.num_points = 10000; % 10000 init
            harris.min_quality_process = 0; 
            harris.num_points_process = 1000; 

            % fast
            fast.min_quality = 0.01;
            fast.num_points = 20000; 

        select_uniform.delta = 8; %radius for non-maximum supression
        select_uniform.nbr_pts = 600;
        select_uniform.viaMatrix_method=true; %true for matrix filling approach, false for keypoints_loc method
            
        ransac.numTrials = 10000; %ransac inside of estimateFundamentalMatrix
        ransac.distanceThreshold = 0.01; 
        ransac.confidence = 99.9; 

        %feature matching
        match.max_ration = 0.6;    %(0,1)
        match.match_threshold = 10; %(0,100)

        % CONTINUOUS OPERATION

        % KLT point Tracker
        klt.NumPyramidLevels = 5; % TODO: nbr was mentioned in lecture
        klt.MaxBidirectionalError = 2; %5 % if inf, is not calculated
        klt.BlockSize = [37 37]; %init 31 31
        klt.MaxIterations = 1000; %init 50
        
        % isClose fct
        is_close.delta = 6; %0.5; %in pixel, treated as same keypoint if within delta

        % p3p 3D-2D algo
        p3p.p3p_and_ransac_iter = 3;
        
            % ransac inside of runP3PandRANSAC
            p3p_ransac.num_iteration = 10000;
            p3p_ransac.pixel_tolerance = 4; % 2 init, better
            p3p_ransac.min_inlier = 6;
            
        p3p.max_delta_loc = 1;
       
        % triangulation
        triang.alpha_threshold = deg2rad(10); % 20 init
        triang.rep_e_threshold = 200; %init 3 % max allowed reprojection error in triangulation
        

        % detect new candidate kp
        processFrame.det_method = bootstrap.det_method;
        
            % harris
            harris.selectUniform = true;
            processFrame.harris.num_points_process = 1000;
            processFrame.harris.min_quality_process = 0.001;
    
end