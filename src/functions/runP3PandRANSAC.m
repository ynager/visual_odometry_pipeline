function [ orient, loc, inlierIdx ] = runP3PandRANSAC( kp_for_p3p, landmarks_for_p3p, cameraParams )
%RUNP3PANDRANSAC should do the same as [orient, loc, inlierIdx] = estimateWorldCameraPose(kp_for_p3p, landmarks_for_p3p, cameraParams)

% get parameters
run('parameters.m');
num_iterations = p3p_ransac.num_iteration;
pixel_tolerance = p3p_ransac.pixel_tolerance;
k = 3;    
min_inlier_count = p3p_ransac.min_inlier_fraction*size(kp_for_p3p,1);

% Initialize RANSAC.
inlierIdx = zeros(size(kp_for_p3p, 1),1);
% (row, col) to (u, v)
% kp_for_p3p = fliplr(kp_for_p3p);
max_num_inliers = 0;
% Replace the following with the path to your camera projection code:

% RANSAC
for i = 1:num_iterations
    [landmark_sample, idx] = datasample(...
        landmarks_for_p3p, k, 1, 'Replace', false);
    keypoint_sample = kp_for_p3p(idx,:);

    % Backproject keypoints to unit bearing vectors.
    normalized_bearings = cameraParams.IntrinsicMatrix\[keypoint_sample'; ones(1, 3)];
    for ii = 1:3
        normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
            norm(normalized_bearings(:, ii), 2);
    end

    poses = p3p(landmark_sample', normalized_bearings);

    % Decode p3p output
    R_C_W_guess = zeros(3, 3, 2);
    t_C_W_guess = zeros(3, 1, 2);
    for ii = 0:1
        R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
        t_W_C_ii = real(poses(:, (1+ii*4)));
        R_C_W_guess(:,:,ii+1) = R_W_C_ii';
        t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
    end
    
    % Count inliers:
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,1) * landmarks_for_p3p') + ...
        repmat(t_C_W_guess(:,:,1), ...
        [1 size(landmarks_for_p3p, 1)]), cameraParams.IntrinsicMatrix);
    difference = kp_for_p3p' - projected_points;
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;
    R_C_W = R_C_W_guess(:,:,1);
    t_C_W = t_C_W_guess(:,:,1);
    
    % If we use p3p, also consider inliers for the alternative solution.
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,2) * landmarks_for_p3p') + ...
        repmat(t_C_W_guess(:,:,2), ...
        [1 size(landmarks_for_p3p, 1)]), cameraParams.IntrinsicMatrix);
    difference = kp_for_p3p' - projected_points;
    errors = sum(difference.^2, 1);
    alternative_is_inlier = errors < pixel_tolerance^2;
    if nnz(alternative_is_inlier) > nnz(is_inlier)
        is_inlier = alternative_is_inlier;
        R_C_W = R_C_W_guess(:,:,2);
        t_C_W = t_C_W_guess(:,:,2);
    end
    
    if nnz(is_inlier) > max_num_inliers && ...
            nnz(is_inlier) >= min_inlier_count
        max_num_inliers = nnz(is_inlier);        
        inlierIdx = is_inlier;
        orient_C_W = R_C_W;
        loc_C_W = t_C_W;
    end
    
end

if max_num_inliers == 0
    orient = [];
    loc = [];
else
   % from C_W to W_C
   orient = orient_C_W';
   loc = -orient_C_W'*loc_C_W;

end

end

