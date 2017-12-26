% Parameters File

%********* dataset *********** (0: KITTI, 1: Malaga, 2: parking)
ds = 2;

%********* debugging printouts *********
debug.print_tracking = true;                                                %for Tracking in processFrame
debug.print_p3p = true;                                                     %for p3p in processFrame (inlier and delta_loc)
debug.print_triangulation = true;                                           % for alpha value based triangulation in processFrame
debug.print_new_landmarks = true;                                           % for nbr of new landmarks in triangulation
debug.print_new_features = true;                                            % for detection of new keypoints
debug.print_lsqnonlin = false;

%********* plot parameters ************
plotParams.plot_p3p_outliers = true; 
plotParams.plot_invalid_ckeypoints = true; 

% interruption after each frame
debug.keyboard_interrupt = false; 

%*********tuning parameters***************
% params for datasets
switch(ds)
   
    %*********************************************************************
    %******  PARKING  ****************************************************    
    case 2
        
        %*********** BOOTSTRAP ******************************************
        
        % BOOTSTRAP images
        bootstrap.init.numTrials = 100;                                     % NO TUNING bootstrap init nbr trials (run bootstrap max numTrial times)
        bootstrap.init.first_location = [1, 0, 0];                          % bootstrap is accepted if close to this location
        bootstrap.images = [1,3];                                           % used bootstrap frames
        
        % feature detection method
        bootstrap.det_method = 'harris';                                    % harris or fast, feature detection method        
            bootstrap.harris.min_quality = 1e-6; %TUNE                      % 0.001 init
            bootstrap.fast.min_quality = 0.01;
            
        % selectUniform / nonMaxSupression
        bootstrap.select_by_nonMax = true;
        % nonMaxSupression
        bootstrap.select_keypoints.delta = 1;  %TUNE                        % online: 8
        bootstrap.select_keypoints.viaMatrix_method = false;                 %true for matrix filling approach, false for keypoints_loc method
        bootstrap.select_keypoints.nbr_pts = 800; %TUNE                    
        % selectUniform
        bootstrap.harris.num_points = 500;                                  % 10000 init
        bootstrap.fast.num_points = 20000;
                
        % feature matching/tracking method 
        bootstrap.use_KLT = true;
            
        % KLT method             
        bootstrap.klt.NumPyramidLevels = 5;                                 % online 5 % TODO: nbr was mentioned in lecture
        bootstrap.klt.MaxBidirectionalError = 0.3;                          %5 % if inf, is not calculated
        bootstrap.klt.BlockSize = [31 31];
        bootstrap.klt.MaxIterations = 2000;
                
        %FEATURE MATCHING method
        bootstrap.desc_method = 'HOG';                                      % HOG or auto, descriptor method
        bootstrap.match.max_ration = 0.6;                                   %(0,1)
        bootstrap.match.match_threshold = 10;                               %(0,100)
        
        % BOOTSTRAP LOOP
        bootstrap.loop.numTrials = 50;
        
        % estimate fundamental matrix (eFm)
        bootstrap.eFm.numTrials = 20;                                       %nbr of estimateFundamentalMatrix runs
        bootstrap.eFm.ransac.numTrials = 10000;                             %ransac inside of estimateFundamentalMatrix
        bootstrap.eFm.ransac.distanceThreshold = 0.05; 
        bootstrap.eFm.ransac.confidence = 99.99999999999999;
        bootstrap.eFm.ransac.inlierRatio = 0.8;
        
        % disambiguate camera pose
        bootstrap.disambiguate.wanted_point_Fraction = 0.9;                 % NO TUNING, only for printout needed
        
        % landmark filter
        bootstrap.triang.radius_threshold = 60;
        bootstrap.triang.min_distance_threshold = 2; 
        bootstrap.triang.num_landmarks_bootstrap = 600; %TUNE
        bootstrap.triang.rep_e_threshold = inf; 
        bootstrap.triang.min_landmark_ratio = 0.70; %0.72;                 % LOOP in bootstrap
        
        % TODO: here params for candidate kp search in bootstrap
        bootstrap.is_close.delta = 1;                                      % currently meaningless
        
        %*********** PROCESS FRAME ****************************************
        
        % KLT point Tracker
        processFrame.klt.NumPyramidLevels = bootstrap.klt.NumPyramidLevels;
        processFrame.klt.MaxBidirectionalError = bootstrap.klt.MaxBidirectionalError; %5 % if inf, is not calculated
        processFrame.klt.BlockSize = bootstrap.klt.BlockSize;
        processFrame.klt.MaxIterations = bootstrap.klt.MaxIterations;
        
        % isClose fct
        processFrame.is_close.delta = 0.1;
        
        % run p3p and ransac for 3D-2D localization
        processFrame.localization.numTrials = 3;
        
        % ransac inside of runP3PandRANSAC
        processFrame.p3p_ransac.num_iteration = 1000;
        processFrame.p3p_ransac.pixel_tolerance = 2;                        % 2 init, better
        processFrame.p3p_ransac.min_inlier = 8;
        processFrame.p3p.max_delta_loc = 1;                                 % max allowed step in loc

        % triangulation
        processFrame.triang.alpha_threshold = [deg2rad(3), deg2rad(40)];    % 20 init
        processFrame.triang.rep_e_threshold = 20;                            %init 3 % max allowed reprojection error in triangulation
        processFrame.triang.radius_threshold = 250;                          % max allowable radius from cam
        processFrame.triang.min_distance_threshold = .2; 
        processFrame.triang.num_landmarks_goal = 150;                       % average number of landmarks to achieve
        processFrame.triang.excess_num_landmarks = 30;                      % constant number of triangulated landmarks.
        processFrame.triang.num_landmarks_margin = 0.5;                     % when landmarks fall below num_landmarks_margin*num_landmarks_goal, new landmarks are triangulated
            
        % detect new candidate kp
        processFrame.max_candidate_keypoints = 4000;                        %no new keypoints are added if above max 
        processFrame.det_method = bootstrap.det_method;
            
        % harris
        processFrame.harris.min_quality_process = 1e-8;
        processFrame.harris.filter_size = 5;                                %must be odd

        % selectUniform / NonMaxSupression
        processFrame.select_by_nonMax = true;
        
        % nonMaxSupression
        processFrame.select_keypoints.delta = 2;                            % online: 8
        processFrame.select_keypoints.nbr_pts = 500;
        processFrame.select_keypoints.viaMatrix_method = true;              %true for matrix filling approach, false for keypoints_loc method    
        processFrame.select_keypoints.sparseMatrix = true;                  %use sparse matrix, only useful if viaMatrix_method==true
       
        % selectUniform
        processFrame.harris.num_points = 600;
            
 
                            
end