function [ projected_points ] = projectLandmarksToSensor( landmark_C_frame, K )
%Attention! Differenct convention in how to fill matrixes!!
%projectLandmarksToSensor takes 3D landmarks as 3xM Matrix: [x;y;z ...] and projects
%them with K
%returns projected_points as 2xM Matrix: [x;y ...]

xp = landmark_C_frame(1,:) ./ landmark_C_frame(3,:);
yp = landmark_C_frame(2,:) ./ landmark_C_frame(3,:);

% convert to pixel coordinates
projected_points = K * [xp; yp; ones(1, length(yp))];
projected_points = projected_points(1:2, :);

end

