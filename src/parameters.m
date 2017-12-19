% Parameters File

%*********dataset***********
% dataset (0: KITTI, 1: Malaga, 2: parking)
ds = 2;

%*********debugging*********
% printouts for debugging
debug.print_tracking = true; %for Tracking in processFrame
debug.print_p3p = true; %for p3p in processFrame (inlier and delta_loc)
debug.print_triangulation = true; % for alpha value based triangulation in processFrame
debug.print_new_landmarks = true; % for nbr of new landmarks in triangulation
debug.print_new_features = true; % for detection of new keypoints

% interruption after each frame
debug.keyboard_interrupt = false; 

%*********tuning parameters***************
% params for datasets
switch(ds)
    
    %********PARKING*********    
    case 2
        
        %***********BOOTSTRAP*********
        
        % BOOTSTRAP images
        bootstrap.init.numTrials = 100; % NO TUNING bootstrap init nbr trials (run bootstrap max numTrial times)
        bootstrap.init.x_interval = [0.99 1.01]; % NO TUNING bootstrap init accepted if x value in this interval   
        bootstrap.images = [1,3]; % used bootstrap frames
        
        % feature detection method
        bootstrap.det_method = 'harris'; % harris or fast, feature detection method        
            bootstrap.harris.min_quality = 1e-6; % 0.001 init
            bootstrap.fast.min_quality = 0.01;
            
        % selectUniform / nonMaxSupression
        bootstrap.select_by_nonMax = true;
            % nonMaxSupression
            bootstrap.select_keypoints.delta = 3; % online: 8
            bootstrap.select_keypoints.nbr_pts = 600; %only for viaMatrix_method
            bootstrap.select_keypoints.viaMatrix_method = true; %true for matrix filling approach, false for keypoints_loc method
            % selectUniform
            bootstrap.harris.num_points = 500; % 10000 init
            bootstrap.fast.num_points = 20000;
                
        % feature matching/tracking method
            % KLT method
            bootstrap.use_KLT = true;             
                % KLT point Tracker
                bootstrap.klt.NumPyramidLevels = 3; % online 5 % TODO: nbr was mentioned in lecture
                bootstrap.klt.MaxBidirectionalError = 0.5; %5 % if inf, is not calculated
                bootstrap.klt.BlockSize = [31 31];
                bootstrap.klt.MaxIterations = 1000;
            % FEATURE MATCHING method
                bootstrap.desc_method = 'HOG'; % HOG or auto, descriptor method
                bootstrap.match.max_ration = 0.6;    %(0,1)
                bootstrap.match.match_threshold = 10; %(0,100)
        
        % estimate fundamental matrix (eFm)
        bootstrap.eFm.numTrials = 1000; %nbr of estimateFundamentalMatrix runs
        bootstrap.eFm.ransac.numTrials = 10000; %ransac inside of estimateFundamentalMatrix
        bootstrap.eFm.ransac.distanceThreshold = 0.01; 
        bootstrap.eFm.ransac.confidence = 99.9;
        bootstrap.eFm.ransac.inlierRatio = 0.3;
        
        % disambiguate camera pose
        bootstrap.disambiguate.wanted_point_Fraction = 0.9; % NO TUNING, only for printout needed
        
        % landmark filter
        bootstrap.triang.radius_threshold = 60;
        bootstrap.triang.num_landmarks_bootstrap = 600;
        
        % TODO: here params for candidate kp search in bootstrap
        bootstrap.is_close.delta = 6;
        
        %***********PROCESSFRAME***********
        
        % KLT point Tracker
        processFrame.klt.NumPyramidLevels = bootstrap.klt.NumPyramidLevels;
        processFrame.klt.MaxBidirectionalError = bootstrap.klt.MaxBidirectionalError; %5 % if inf, is not calculated
        processFrame.klt.BlockSize = bootstrap.klt.BlockSize;
        processFrame.klt.MaxIterations = bootstrap.klt.MaxIterations;
        
        % isClose fct
        processFrame.is_close.delta = 6;
        
        % run p3p and ransac for 3D-2D localization
        processFrame.localization.numTrials = 3;
        
            % ransac inside of runP3PandRANSAC
            processFrame.p3p_ransac.num_iteration = 2000;
            processFrame.p3p_ransac.pixel_tolerance = 0.5;       % 2 init, better
            processFrame.p3p_ransac.min_inlier = 8;
            
        %max allowed step in loc
        processFrame.p3p.max_delta_loc = 1;

        % triangulation
        processFrame.triang.alpha_threshold = [deg2rad(3), deg2rad(30)]; % 20 init
        processFrame.triang.rep_e_threshold = 0.5; %init 3 % max allowed reprojection error in triangulation
        processFrame.triang.radius_threshold = 60; % max allowable radius from cam
        processFrame.triang.num_landmarks = 300;

        % detect new candidate kp
        processFrame.det_method = bootstrap.det_method;
            % harris
            processFrame.harris.min_quality_process = 1e-6;

        % selectUniform / NonMaxSupression
        processFrame.select_by_nonMax = true;
            % nonMaxSupression
            processFrame.select_keypoints.delta = 3; % online: 8
            processFrame.select_keypoints.nbr_pts = 600;
            processFrame.select_keypoints.viaMatrix_method = true;%true for matrix filling approach, false for keypoints_loc method
                %use sparse matrix, only useful if viaMatrix_method==true
                processFrame.select_keypoints.sparseMatrix = true;
            % selectUniform
            processFrame.harris.num_points = 500;
                            
end