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
    
    %if alpha above threshold, do triangulation
    if alpha > alpha_threshold
        
        % get M
        M1 = cameraParams.IntrinsicMatrix * [R1, -T1]; 
        
        % TODO: maybe use reprojectionErrors as decision hot to proceed?
        % triangulate
        [xyzPoints, reprojectionErrors] = triangulate(currState.candidate_kp(i,:), currState.first_obs(i,:), M2', M1');
        
        % save to current State and globalData
        currState.keypoints = [currState.keypoints; currState.candidate_kp(i,:)];
        currState.landmarks = [currState.landmarks; xyzPoints];
        globalData.landmarks = [globalData.landmarks; xyzPoints];
        
        success(i)=true;
    end
end
fprintf('\nTriangulation, created new landmarks: %d',sum(success));
%delete used candidates
currState.candidate_kp(success,:) = [];
currState.first_obs(success,:) = [];
currState.pose_first_obs(success,:) = [];

end

