function [P,] = triangulation_wrapper(E, p1, p2,cameraParams)
%TRIANGULATION_WRAPPER Summary of this function goes here
%   Detailed explanation goes here

K = cameraParams.IntrinsicMatrix;
p1 = [p1.Location';ones(1,length(p1))];
p2 = [p2.Location';ones(1,length(p2))];
% Extract the relative camera positions (R,T) from the essential matrix

% Obtain extrinsic parameters (R,t) from E
[Rots,u3] = decomposeEssentialMatrix(E);

% Disambiguate among the four possible configurations
[R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1,p2,K,K);

% Triangulate a point cloud using the final transformation (R,T)
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];
P = linearTriangulation(p1,p2,M1,M2);
P = P(1:3,:);
P = P';
end

