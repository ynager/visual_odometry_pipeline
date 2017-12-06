% Parameters File

% dataset (0: KITTI, 1: Malaga, 2: parking)
ds = 2;

% bootstrap images
bootstrap.images = [1,3]; 
bootstrap.det_method = 'harris';

    % harris
    harris.min_quality = 0.1;
    harris.num_points = 150; 