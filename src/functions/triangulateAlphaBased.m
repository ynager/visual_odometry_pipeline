function [ currState, globalData ] = triangulateAlphaBased( currState, cameraParams, currRT, globalData )
%TRIANGULATEALPHABASED checks for all candidate keypoints if bearing
%vektors have a angle bigger than alpha, if yes the triangulation is done
%and the landmark and keypoint is stored into currState and deleted in
%candidate list (and first obs and pose of first obs)

% get parameters
run('parameters.m');

% calc constant R and T for current pose
R2 = currRT(:,1:3);
T2 = currRT(:,4);
% current M
M2 = cameraParams.IntrinsicMatrix * [R2, -T2];

%calc current bearing vector
bearings_curr = getBearingVector( currState.candidate_kp, cameraParams.IntrinsicMatrix );
bearings_curr_W = R2*bearings_curr';

%loop through first oberservations and calc bearing and check alpha
bearings_prev = getBearingVector( currState.first_obs, cameraParams.IntrinsicMatrix );

%used for deleting candidates later
success = logical(zeros(size(currState.pose_first_obs,1),1));

if debug.print_triangulation
    book_alpha = zeros(size(currState.pose_first_obs,1),1);
    book_alpha_high = zeros(size(currState.pose_first_obs,1),1);
    book_rep_e = zeros(size(currState.pose_first_obs,1),1);
end

reprojection_errors = zeros(size(currState.pose_first_obs,1),1);
unfiltered_landmarks = zeros(size(currState.pose_first_obs,1),3);

for i = 1:size(currState.pose_first_obs,1)
    
    %get bearing and R and T
    bear = bearings_prev(i,:);
    R1 = currState.pose_first_obs(i,1:9);
    R1 = reshape(R1,3,3);
    T1 = reshape(currState.pose_first_obs(i,10:end),3,1);
    
    bearings_prev_W = R1*bear'; %attention change of convention of axes here
    
    %TODO: test if angle is always small or vectors potentially can swap
    %test alpha
    alpha = atan2(norm(cross(bearings_prev_W,bearings_curr_W(:,i))),dot(bearings_prev_W,bearings_curr_W(:,i)));
    
    if debug.print_triangulation
        book_alpha(i) = alpha;
    end
    
    %if alpha above threshold, do triangulation
    if alpha > triang.alpha_threshold && alpha < deg2rad(25)
        
        % get M
        M1 = cameraParams.IntrinsicMatrix * [R1, -T1]; 
        
        % TODO: maybe use reprojectionErrors as decision hot to proceed?
        % triangulate
        [xyzPoints, reprojectionErrors] = triangulate(currState.first_obs(i,:),currState.candidate_kp(i,:), M1', M2');
        
        if reprojectionErrors > triang.rep_e_threshold %skip this triangulation if reprojection error too high
        	continue 
        end
        
        % Fill in unfiltered values of every triangulated point 
        unfiltered_landmarks(i,:) = xyzPoints; 
        reprojection_errors(i) = reprojectionErrors; 
        
        if debug.print_triangulation
            book_rep_e(i) = reprojectionErrors;
            book_alpha_high(i) = alpha;
        end
        
        % save to current State and globalData
        %currState.keypoints = [currState.keypoints; currState.candidate_kp(i,:)];
        %currState.landmarks = [currState.landmarks; xyzPoints];
        %globalData.landmarks = [globalData.landmarks; xyzPoints];
        
        success(i)=true;
    end
end

% filter landmarks to get only the best num_landmarks inside
% landmark_radius
fprintf('\nnumber of unfiltered landmarks: %d\n',length(unfiltered_landmarks));
fprintf('sum success: %d\n',sum(success)); 
[xyzPoints_filt, ind_filt] = getFilteredLandmarks(unfiltered_landmarks, reprojection_errors, R2, T2, triang.radius_threshold, triang.num_landmarks);    
fprintf('number of filtered landmarks: %d\n',length(xyzPoints_filt));

% save to current State and globalData
currState.keypoints = [currState.keypoints; currState.candidate_kp(ind_filt,:)]; 
currState.landmarks = [currState.landmarks; xyzPoints_filt]; 
globalData.landmarks = [globalData.landmarks; xyzPoints_filt]; 

if debug.print_triangulation
    fprintf('\nTriangulation, mean alpha high: %.2f',rad2deg(mean(book_alpha_high(success))));
    fprintf('\nTriangulation, min alpha high: %.2f',rad2deg(min(book_alpha_high(success))));
    fprintf('\nTriangulation, max alpha high: %.2f',rad2deg(max(book_alpha_high(success))));
    
    fprintf('\nTriangulation, mean alpha: %.2f',rad2deg(mean(book_alpha)));
    fprintf('\nTriangulation, min alpha: %.2f',rad2deg(min(book_alpha)));
    fprintf('\nTriangulation, max alpha: %.2f',rad2deg(max(book_alpha)));
    
    fprintf('\nTriangulation, mean reprojE: %.2f',mean(book_rep_e(success)));
    fprintf('\nTriangulation, min reprojE: %.2f',min(book_rep_e(success)));
    fprintf('\nTriangulation, max reprojE: %.2f',max(book_rep_e(success))); 
%     figure(3)
%     histogram(book_rep_e(success),'BinWidth',0.5)
end

if debug.print_new_landmarks
    fprintf('\nTriangulation, created new landmarks: %d',sum(success));
end
%delete used candidates
currState.candidate_kp(ind_filt,:) = [];
currState.first_obs(ind_filt,:) = [];
currState.pose_first_obs(ind_filt,:) = [];

end