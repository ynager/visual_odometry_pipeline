function [xyzpoints, ind] = getFilteredLandmarks(xyzpoints, reprError, R, T, max_radius, min_distance_threshold, num_landmarks)
% Filters triangulated landmarks such that only a number of num_landmarks 
% survive that lie within max_radius from the current camera pose and do 
% not lie behind the camera

%translate and rotate points into camera frame
% c_points = (xyzpoints - T')*R';
% alternative:
c_points = (R'*(xyzpoints - T')')';


%find points outside of max_radius and behind camera
outliers = any([(c_points(:,3) < min_distance_threshold)'; ... 
    (sqrt(c_points(:,1).^2 + c_points(:,2).^2 + c_points(:,3).^2) > max_radius)'],1);

inliers_ind = find(~outliers);

%get k elements with lowest reprError that do fulfil above condition
[~,minind] = mink(reprError(~outliers), num_landmarks,1);

%get final filtered indices and landmarks
ind = inliers_ind(minind); 

xyzpoints = xyzpoints(ind,:); 
end

