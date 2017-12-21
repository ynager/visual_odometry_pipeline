function [xyzpoints, ind_filt, ind_invalid] = getFilteredLandmarks(xyzpoints, reprError, R, T, max_radius, min_distance_threshold, rep_e_threshold, num_landmarks)
% Filters triangulated landmarks such that only a number of num_landmarks 
% survive that lie within max_radius from the current camera pose and do 
% not lie behind the camera

%translate and rotate points into camera frame
% c_points = (xyzpoints - T')*R';
% alternative:
c_points = (R'*(xyzpoints - T')')';

%find points outside of max_radius and behind camera
invalid = any([(c_points(:,3) < min_distance_threshold)'; ... 
    (sqrt(c_points(:,1).^2 + c_points(:,2).^2 + c_points(:,3).^2) > max_radius)'; ...
    reprError' > rep_e_threshold],1);


%get indices of valid/invalid points
ind_invalid = find(invalid); 
ind_valid = find(~invalid);

%get k elements with lowest reprError that do fulfil above condition
[~,min_error_ind] = mink(reprError(ind_valid), num_landmarks,1);
meana = mean(reprError(:)); 
meanv = mean(reprError(min_error_ind));
disp(meana); 

%get final filtered indices and landmarks
ind_filt = ind_valid(min_error_ind); 

xyzpoints = xyzpoints(ind_filt,:); 
end

