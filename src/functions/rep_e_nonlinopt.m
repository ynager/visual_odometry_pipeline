function [rep_e] = rep_e_nonlinopt(RT_twist,keypoints, landmarks, cameraParams)
%REP_E_NONLINOPT takes in R and T as twist, projects the landmakrs onto the
%sensor based on R T and K, and calculates the reprojection error
%   R und T sind ..._W_C, landmarks also in world frame
% note: code initially from runP3PandRANSAC
%
% input: RT_twist as (6,1)
%        keypoints as [u_hor v_vert;...]
%        landmarks as [X Y Z;...]
%        cameraParams object

% get R T from twist, as ..._W_C
RT_homo = twist2HomogMatrix(RT_twist);
R_W_C = RT_homo(1:3,1:3);
t_W_C = RT_homo(1:3,4);

% from ..._W_C to ..._C_W
R_C_W = R_W_C';
t_C_W = -R_C_W*t_W_C;

% project landmarks in C frame (rot from W to C) onto sensor
projected_points = projectLandmarksToSensor(...
    (R_C_W * landmarks') + ...
    repmat(t_C_W, ...
    [1 size(landmarks, 1)]), cameraParams.IntrinsicMatrix);

%ger reproj error
difference = keypoints' - projected_points;
rep_e = sqrt(sum(difference.^2, 1)');



end

