function [R,T] = disambiguateRelativePose(Rots,u3,inlierPoints_0,inlierPoints_1,cameraParams)
% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   inlierPoints_0   -  cornerPoints object with matched inliers in image 0
%   inlierPoints_1   -  cornerPoints object with matched inliers in image 1
%   cameraParams     -  camera parameter object
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C1_C0 = T_C1_W is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 0)
%   to camera 1.
%

K0 = cameraParams.IntrinsicMatrix; 
K1 = cameraParams.IntrinsicMatrix; 

M0 = K0 * eye(3,4); % Projection matrix of camera 1

total_points_in_front_best = 0;
for iRot = 1:2
    R_C1_C0_test = Rots(:,:,iRot);
    
    for iSignT = 1:2
        T_C1_C0_test = u3 * (-1)^iSignT;
        
        M1 = K1 * [R_C1_C0_test, T_C1_C0_test];
        P_C0 = triangulate(inlierPoints_0,inlierPoints_1,M0',M1'); 
        
        % Make homogenous
        P_C0 = cat(2,P_C0, ones(length(P_C0),1)); 
        
        % project in both cameras
        P_C1 = [R_C1_C0_test T_C1_C0_test] * P_C0';
        
        num_points_in_front0 = sum(P_C0(3,:) > 0);
        num_points_in_front1 = sum(P_C1(3,:) > 0);
        total_points_in_front = num_points_in_front0 + num_points_in_front1;
              
        if (total_points_in_front > total_points_in_front_best)
            % Keep the rotation that gives the highest number of points
            % in front of both cameras
            R = R_C1_C0_test;
            T = T_C1_C0_test;
            total_points_in_front_best = total_points_in_front;
        end
    end
end

end

