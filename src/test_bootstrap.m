%%%%%%%************preparation for testing***********
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
R0 = eye(3);
T0 = zeros(3,1);
%create camera 2 on the right of cam 1 and slightly turned towards of cam 1
R1 = roty(-45); %5 does NOT run %y points downwards
T1 = [1; 0; 0.5]; %to the right and bit to the front

%define K form ds=0
K = [7.188560000000e+02 0 6.071928000000e+02
    0 7.188560000000e+02 1.852157000000e+02
    0 0 1];

%create keypoints
landmarks_C_frame_1 = (R1')*(landmarks)-(R1')*T1; %formula is validated! works!

kp_0 = projectLandmarksToSensor( landmarks, K );
kp_1 = projectLandmarksToSensor( landmarks_C_frame_1, K );


%change notation for testing
matchedPoints_0 = cornerPoints(kp_0');
matchedPoints_1 = cornerPoints(kp_1');
cameraParams = cameraParameters('IntrinsicMatrix', K);
landmarks = landmarks';
%%%%%%%%%%%%%%************finish preparation************







%%%%%**********************TESTING OF BOOTSTRAP************
   
% ESTIMATE FUNDAMENTAL MATRIX
for i = 1:bootstrap.eFm.numTrials
    % this function uses RANSAC and the 8-point algorithm

%     [F, inlierIdx] = estimateFundamentalMatrix(matchedPoints_0, matchedPoints_1, ...
%                  'Method','RANSAC', 'DistanceThreshold', bootstrap.eFm.ransac.distanceThreshold, ... 
%                  'Confidence',bootstrap.eFm.ransac.confidence, 'NumTrials', bootstrap.eFm.ransac.numTrials);

	[F,inlierIdx]=estimateFundamental_RANSAC(matchedPoints_0, matchedPoints_1,bootstrap.eFm.ransac.distanceThreshold,bootstrap.eFm.ransac.numTrials);

    % Make sure we get enough inliers
    ratio = sum(inlierIdx) / numel(inlierIdx); 
    if(ratio > bootstrap.eFm.ransac.inlierRatio)
        fprintf('Fraction of inliers for F: %.2f',ratio);
        break;
    elseif i==bootstrap.eFm.numTrials
        display('max iterations in estimateFundamentalMatrix trials reached without success, bad F is likely')
    end
end

% get essential matrix (must be transposed for relativeCameraPose)
E = cameraParams.IntrinsicMatrix'*F*cameraParams.IntrinsicMatrix;

% Get the inlier points in each image
inlierPoints_0 = matchedPoints_0(inlierIdx, :);
inlierPoints_1 = matchedPoints_1(inlierIdx, :);

% % Compute the camera pose from the fundamental matrix and disambiguate
% % invalid configurations using inlierPoints
% [orient, loc, validPointFraction] = ...
%         relativeCameraPose(E, cameraParams, inlierPoints_0, inlierPoints_1);    

% Obtain extrinsic parameters (R,t) from E
[Rots,u3] = decomposeEssentialMatrix(E);

% Disambiguate among the four possible configurations
p0 = [inlierPoints_0.Location ones(length(inlierPoints_0.Location),1)]';
p1 = [inlierPoints_1.Location ones(length(inlierPoints_1.Location),1)]';
K = cameraParams.IntrinsicMatrix;
[orient_C_W,loc_C_W] = disambiguateRelativePose(Rots,u3,p0,p1,K,K);
orient = orient_C_W';
loc = -orient*loc_C_W;

%%%%
loc = loc*norm(T1);
%%%%
fprintf('\nEstimated Location: x=%.2f  y=%.2f  z=%.2f',loc(:));

% if(validPointFraction < bootstrap.disambiguate.wanted_point_Fraction) 
%     fprintf('\nSmall fraction of valid points when running relativeCameraPose. Essential Matrix might be bad.\n'); 
% end

%% Triangulate to get 3D points

% %%%%DEBUG%%%%%
% orient=orient'; %wtf
% loc = loc'; %only vector stuff...
% %%%%%%%%%%%%%%

% get rotation matrix and translation vector from pose orientation and location    
%%%%%%%%?????????
R = orient';
t = -R*loc;

% calculate camera matrices
M1 = cameraParams.IntrinsicMatrix * eye(3,4); 
M2 = cameraParams.IntrinsicMatrix * [R, t];
%M1 = eye(4,3)*cameraParams.IntrinsicMatrix; 
%M2 = [R;t]*cameraParams.IntrinsicMatrix;
%%%%%%%%?????????

% triangulate
[xyzPoints, reprojectionErrors] = triangulate(inlierPoints_0, inlierPoints_1, M1', M2');

%%%%%%%%%**************END TESTING**************



%PRINTOUTS
display('norm of difference between rotmatrices')
display(R1-orient)

display('norm of difference between locations')
display(T1-loc)

diff = xyzPoints-landmarks;
diff_norm = sqrt(diff(:,1).^2+diff(:,2).^2+diff(:,3).^2);
display('norm of difference between original and projected landmarks')
display(diff_norm)


