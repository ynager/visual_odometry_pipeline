function [xyzpoints, ind_filt, ind_invalid] = getFilteredLandmarks(xyzpoints, reprError, R, T, max_radius, min_distance_threshold, rep_e_threshold, num_landmarks)
% Filters triangulated landmarks such that only a number of num_landmarks 
% survive that lie within max_radius from the current camera pose and do 
% not lie behind the camera

%translate and rotate points into camera frame
% c_points = (xyzpoints - T')*R';
% alternative:
c_points = (R'*(xyzpoints - T')')';

invalid_1 = c_points(:,3) < min_distance_threshold; 
invalid_2 = (c_points(:,1).^2 + c_points(:,2).^2 + c_points(:,3).^2) > max_radius^2; 
invalid_3 = reprError > rep_e_threshold; 
 
invalid = invalid_3 | invalid_2 | invalid_1;
 
fprintf('\n\n# triang invalid due to distance behind threshold: %d\n', sum(invalid_1)); 
fprintf('# triang invalid due to exceeding max radius: %d\n', sum(invalid_2)); 
fprintf('# triang invalid due to repr error above threshold: %d\n\n', sum(invalid_3)); 


%get indices of valid/invalid points
ind_invalid = find(invalid); 
ind_valid = find(~invalid);

%get k elements with lowest reprError that do fulfil above condition
[~,min_error_ind] = mink(reprError(ind_valid), num_landmarks,1);

%get final filtered indices and landmarks
ind_filt = ind_valid(min_error_ind); 

xyzpoints = xyzpoints(ind_filt,:); 
end

