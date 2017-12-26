function [ currState, globalData ] = triangulateAlphaBased( currState, cameraParams, currRT, globalData )
%TRIANGULATEALPHABASED checks for all candidate keypoints if bearing
%vektors have a angle bigger than alpha, if yes the triangulation is done
%and the landmark and keypoint is stored into currState and deleted in
%candidate list (and first obs and pose of first obs)

% get parameters
run('parameters.m');

% set debug data to zero
globalData.debug.ckeypoints_invalid = [0, 0];

% check if current number of landmarks is larger than goal number * margin
margin = processFrame.triang.num_landmarks_margin; 
if(length(currState.landmarks) > margin*processFrame.triang.num_landmarks_goal)
    return; 
end

% calc constant R and T for current pose
R2 = currRT(:,1:3);
T2 = currRT(:,4);
% current M
M2 = cameraParams.IntrinsicMatrix * [R2, -T2];

%%%%%%???????????
% M2 = cameraParams.IntrinsicMatrix * [R2', -R2'*T2];
%%%%%%???????????

%calc current bearing vector
bearings_curr = getBearingVector( currState.candidate_kp, cameraParams.IntrinsicMatrix );
bearings_curr_W = R2*bearings_curr';

%loop through first oberservations and calc bearing and check alpha
bearings_prev = getBearingVector( currState.first_obs, cameraParams.IntrinsicMatrix );

if debug.print_triangulation
    book_alpha = zeros(size(currState.pose_first_obs,1),1);
    book_alpha_high = zeros(size(currState.pose_first_obs,1),1);
    book_rep_e = zeros(size(currState.pose_first_obs,1),1);
end

reprojection_errors = 1000*ones(size(currState.pose_first_obs,1),1);
unfiltered_landmarks = zeros(size(currState.pose_first_obs,1),3);
alpha_ok = logical(zeros(size(currState.pose_first_obs,1),1));

for i = 1:size(currState.pose_first_obs,1)
    
    %get bearing and R and T
    bear = bearings_prev(i,:);
    R1 = currState.pose_first_obs(i,1:9);
    R1 = reshape(R1,3,3);
    T1 = reshape(currState.pose_first_obs(i,10:end),3,1);
    
    bearings_prev_W = R1*bear'; %attention change of convention of axes here
    
    % get bearing angle
    alpha = atan2(norm(cross(bearings_prev_W,bearings_curr_W(:,i))),dot(bearings_prev_W,bearings_curr_W(:,i)));
    
    if debug.print_triangulation
        book_alpha(i) = alpha;
    end
    
    %if alpha above threshold, do triangulation
    if alpha > processFrame.triang.alpha_threshold(1) && alpha < processFrame.triang.alpha_threshold(2)
        
        alpha_ok(i) = true; 
        
        % get M
        M1 = cameraParams.IntrinsicMatrix * [R1, -T1]; 
        
        % TODO: maybe use reprojectionErrors as decision hot to proceed?
        % triangulate
        [xyzPoints, reprojectionErrors] = triangulate(currState.first_obs(i,:),currState.candidate_kp(i,:), M1', M2');
        
        % Fill in unfiltered values of every triangulated point 
        unfiltered_landmarks(i,:) = xyzPoints; 
        reprojection_errors(i) = reprojectionErrors;
        
        if debug.print_triangulation
            book_rep_e(i) = reprojectionErrors;
            book_alpha_high(i) = alpha;
        end
             
    end
end

% calculate how many new landmarks should be generated
offset = processFrame.triang.excess_num_landmarks; 
num_new_landmarks = ceil(offset + (processFrame.triang.num_landmarks_goal - length(currState.keypoints)));

% filter landmarks to get only the best num_landmarks inside
unfiltered_landmarks = unfiltered_landmarks(alpha_ok,:); 
reprojection_errors = reprojection_errors(alpha_ok); 

[xyzPoints_filt, ind_filt, ind_invalid, ratio] = ... 
    getFilteredLandmarks(unfiltered_landmarks, reprojection_errors, R2, T2, processFrame.triang.radius_threshold, processFrame.triang.min_distance_threshold,processFrame.triang.rep_e_threshold, num_new_landmarks);    

% get indices relative to total array back
ind_alpha_ok = find(alpha_ok);
ind_global_filt = ind_alpha_ok(ind_filt); 
ind_global_invalid = ind_alpha_ok(ind_invalid); 

% save to current State and globalData
currState.keypoints = [currState.keypoints; currState.candidate_kp(ind_global_filt,:)]; 
currState.landmarks = [currState.landmarks; xyzPoints_filt]; 
globalData.landmarks = [globalData.landmarks; xyzPoints_filt]; 

%calculate indices to be removed
ind_global_delete = union(ind_global_invalid, ind_global_filt); 

% Add debug data
globalData.debug.ckeypoints_invalid = currState.candidate_kp(ind_global_invalid,:);

%delete used candidates
currState.candidate_kp(ind_global_delete,:) = [];
currState.first_obs(ind_global_delete,:) = [];
currState.pose_first_obs(ind_global_delete,:) = [];



if debug.print_triangulation
    fprintf('\nTriangulation, mean alpha high: %.2f',rad2deg(mean(book_alpha_high(ind_global_filt))));
    fprintf('\nTriangulation, min alpha high: %.2f',rad2deg(min(book_alpha_high(ind_global_filt))));
    fprintf('\nTriangulation, max alpha high: %.2f',rad2deg(max(book_alpha_high(ind_global_filt))));
    
    fprintf('\nTriangulation, mean alpha: %.2f',rad2deg(mean(book_alpha)));
    fprintf('\nTriangulation, min alpha: %.2f',rad2deg(min(book_alpha)));
    fprintf('\nTriangulation, max alpha: %.2f',rad2deg(max(book_alpha)));
    
    fprintf('\nTriangulation, mean reprojE: %.2f',mean(book_rep_e(ind_global_filt)));
    fprintf('\nTriangulation, min reprojE: %.2f',min(book_rep_e(ind_global_filt)));
    fprintf('\nTriangulation, max reprojE: %.2f',max(book_rep_e(ind_global_filt))); 
%     figure(3)
%     histogram(book_rep_e(success),'BinWidth',0.5)
end

if debug.print_new_landmarks
    fprintf('\nTriangulation, created new landmarks: %d',length(ind_filt));
end

end