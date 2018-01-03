function [xyzpoints, ind_filt, ind_invalid, ratio] = getFilteredLandmarks(xyzpoints, keypoints, reprError, R, T, triangparams, cameraParams)
% Filters triangulated landmarks such that only a number of num_landmarks 
% survive that lie within max_radius from the current camera pose and do 
% not lie behind the camera

% get params
max_radius = triangparams.radius_threshold;
min_distance_threshold = triangparams.min_distance_threshold;
rep_e_threshold = triangparams.rep_e_threshold;  

%translate and rotate points into camera frame
c_points = (R')*(xyzpoints')-(R')*T;
c_points = c_points';

invalid_1 = c_points(:,3) < min_distance_threshold; 
invalid_2 = (c_points(:,1).^2 + c_points(:,2).^2 + c_points(:,3).^2) > max_radius^2; 
invalid_3 = reprError > rep_e_threshold; 
 
invalid = invalid_3 | invalid_2 | invalid_1;
 
fprintf('\n\n# triang invalid due to distance behind threshold: %d\n', sum(invalid_1)); 
fprintf('# triang invalid due to exceeding max radius: %d\n', sum(invalid_2)); 
fprintf('# triang invalid due to repr error above threshold: %d\n', sum(invalid_3)); 


%get indices of valid/invalid points
ind_invalid = find(invalid); 
ind_valid = find(~invalid);

%get ratio of useful landmarks
ratio = length(ind_valid)/length(invalid);

if triangparams.usegrid
    max_landmarks_per_bin = triangparams.max_landmarks_per_bin;
    
    n_bins = 10;
    edges_x = 1:cameraParams.ImageSize(2)/n_bins:cameraParams.ImageSize(2); 
    edges_y = 1:cameraParams.ImageSize(1)/n_bins:cameraParams.ImageSize(1); 

    Cx = discretize(keypoints(ind_valid,1), edges_x); 
    Cy = discretize(keypoints(ind_valid,2), edges_y);
    min_error_ind = []; 
    for i = 1:n_bins
        for j = 1:n_bins
            ind_box = find(and(Cx == i, Cy == j));
            n_in_box = length(ind_box); 
            n_landmarks = max_landmarks_per_bin - n_in_box;
            if(n_landmarks < 0)
    %               [~,minind] = mink(reprError(ind_box), n_landmarks,1);
    %               min_error_ind = [min_error_ind, ind_box(minind)'];
                [~,kill_idx] = datasample(ind_box, abs(n_landmarks),'Replace',false);
                ind_box(kill_idx)=[];
 
            end
            min_error_ind = [min_error_ind, ind_box'];
        end
    end


    %get k elements with lowest reprError that do fulfil above condition
    %[~,min_error_ind] = mink(reprError(ind_valid), num_landmarks,1);
    fprintf('# selected %d from %d landmarks\n\n', length(min_error_ind), length(xyzpoints)); 

    %get final filtered indices and landmarks

    ind_filt = ind_valid(min_error_ind); 
else
    ind_filt = ind_valid;
end
xyzpoints = xyzpoints(ind_filt,:); 
end

