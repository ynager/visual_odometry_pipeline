%%%%%%%%%%%************prepare testing************
clear all;
close all;

addpath('plot')
addpath('functions')
addpath('functions/NonLinLS')
addpath('functions/triangulation')
run('parameters.m');

%create random landmarks
nbr_landmarks = 50;
min_z = 5;
max_z = 10;
rand_z = min_z + (max_z-min_z).*rand(nbr_landmarks,1);
min_xy = -0.5;
max_xy = 0.5;
rand_x = min_xy + (max_xy-min_xy).*rand(nbr_landmarks,1);
rand_y = min_xy + (max_xy-min_xy).*rand(nbr_landmarks,1);

landmarks = [rand_x, rand_y, rand_z]'; %note: different convention as we use in code!

%create camera 1
R1_t = roty(5);
T1_t = [-1; 0; 0];
%create camera 2 on the right of cam 1 and slightly turned towards of cam 1
R2_t = roty(-45); %5 does NOT run %y points downwards
T2_t = [1; 0; 0.5]; %to the right and bit to the front

%define K form ds=0
K = [7.188560000000e+02 0 6.071928000000e+02
    0 7.188560000000e+02 1.852157000000e+02
    0 0 1];

%create keypoints
landmarks_C_frame_1 = (R1_t')*(landmarks)-(R1_t')*T1_t;
landmarks_C_frame_2 = (R2_t')*(landmarks)-(R2_t')*T2_t; %formula is validated! works!

kp_1 = projectLandmarksToSensor( landmarks_C_frame_1, K ); % transp needed for convention
kp_2 = projectLandmarksToSensor( landmarks_C_frame_2, K );


%change notation for testing
currRT = [R2_t T2_t];
currState.candidate_kp = kp_2';
currState.first_obs = kp_1';
cameraParams = cameraParameters('IntrinsicMatrix', K);
% landmarks = landmarks';
%%%%%%%%%%%***************finish preparation*************









%%%%%%%%%%%%%%%%%************TRIANGULATE ALPHA BASED*************

% calc constant R and T for current pose
R2 = currRT(:,1:3);
T2 = currRT(:,4);
%cheat
R1 = R1_t;
T1 = T1_t;

% current M
%%%%%%???????????
% M2 = cameraParams.IntrinsicMatrix * [R2, T2];
M2 = cameraParams.IntrinsicMatrix * [R2', -R2'*T2];
%%%%%%???????????

%calc current bearing vector
bearings_curr = getBearingVector( currState.candidate_kp, cameraParams.IntrinsicMatrix );
bearings_curr_W = R2*bearings_curr';

%loop through first oberservations and calc bearing and check alpha
bearings_prev = getBearingVector( currState.first_obs, cameraParams.IntrinsicMatrix );

if debug.print_triangulation
    book_alpha = zeros(size(currState.candidate_kp,1),1);
    book_alpha_high = zeros(size(currState.candidate_kp,1),1);
    book_rep_e = zeros(size(currState.candidate_kp,1),1);
end

reprojection_errors = 1000*ones(size(currState.candidate_kp,1),1);
unfiltered_landmarks = zeros(size(currState.candidate_kp,1),3);
alpha_ok = logical(zeros(size(currState.candidate_kp,1),1));

for i = 1:size(currState.candidate_kp,1)
    
    %get bearing and R and T
    bear = bearings_prev(i,:);
%     R1 = currState.pose_first_obs(i,1:9);
%     R1 = reshape(R1,3,3);
%     T1 = reshape(currState.pose_first_obs(i,10:end),3,1);
    
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
        % ???????????
        %M1 = cameraParams.IntrinsicMatrix * [R1, -T1];
        M1 = cameraParams.IntrinsicMatrix * [R1', -R1'*T1]; 
        % ???????????
        
        % triangulate 
        [xyzPoints, reprojectionErrors] = triangulate(currState.first_obs(i,:),currState.candidate_kp(i,:), M1', M2');
        
        % Fill in unfiltered values of every triangulated point 
        unfiltered_landmarks(i,:) = xyzPoints; 
        reprojection_errors(i) = reprojectionErrors;
        
        if debug.print_triangulation
            book_rep_e(i) = reprojectionErrors;
            book_alpha_high(i) = alpha;
        end
    else
        display('alpha too small')
        alpha
    end
end
%%%%%%%%***********FINISH TESTING



%PRINTOUTS
diff = unfiltered_landmarks-landmarks';
diff_norm = sqrt(diff(:,1).^2+diff(:,2).^2+diff(:,3).^2);
display('norm of difference between original and projected landmarks')
display(diff_norm)

display('reproj error')
display(reprojection_errors)