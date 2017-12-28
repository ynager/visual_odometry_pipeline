function updateCamTrajectoryPlot(viewId, globalData, currState, I, plotHandles, plotParams)
warning off;  

sf = globalData.scale_factor; 

% axis 1 ****************************************************
axes(plotHandles.axes1);
% Plot estimated camera location and orientation
loc = globalData.vSet.Views.Location{viewId}*sf; 
orient = globalData.vSet.Views.Orientation{viewId};
heading = orient * [0, 0, 1]'; 
heading = 10*heading/norm(heading); 
camh = heading + loc'; 

h = plotHandles.axes1.Children(6);
set(h, 'XData', loc(1), 'YData', loc(2), 'ZData', loc(3));
h = plotHandles.axes1.Children(5);
set(h, 'XData', [loc(1), camh(1)], 'YData',[loc(2), camh(2)], 'ZData', [loc(3), camh(3)]);

% Plot the estimated trajectory
h = plotHandles.axes1.Children(4);
locations = cat(1, globalData.vSet.Views.Location{:})*sf;
set(h, 'XData', locations(:,1), 'YData', ...
    locations(:,2), 'ZData', locations(:,3));

% Updata point cloud
h = plotHandles.axes1.Children(2);
global_landmarks = globalData.landmarks*sf; 
set(h, 'XData', global_landmarks(:,1), 'YData', ...
    global_landmarks(:,2), 'ZData', global_landmarks(:,3));

% Updata inlier point cloud
h = plotHandles.axes1.Children(1);
curr_landmarks = currState.landmarks*sf; 
set(h, 'XData', curr_landmarks(:,1), 'YData', ...
    curr_landmarks(:,2), 'ZData', curr_landmarks(:,3));

% set axis limits 5 meters larger than data
limsx=get(plotHandles.axes1,'XLim');
set(plotHandles.axes1,'Xlim',[loc(1)-50, loc(1)+50]); 

limsy=get(plotHandles.axes1,'YLim');
set(plotHandles.axes1,'Ylim',[loc(2)-50, loc(2)+50]); 

limsz=get(plotHandles.axes1,'ZLim');
set(plotHandles.axes1,'Zlim',[loc(3)-50, loc(3)+50]); 

% axes 2 ****************************************************
axes(plotHandles.axes2);   
imshow(I); 
num_inliers = length(globalData.vSet.Views.Points{viewId}); 
title(['Number of inliers: ',num2str(num_inliers)]);
hold on; 

%draw points
scatter(currState.candidate_kp(:,1), currState.candidate_kp(:,2), 5, 'blue', 'filled', 'Marker', 'o'); 
scatter(currState.keypoints(:,1), currState.keypoints(:,2), 5, 'green', 'Marker', '+'); 

%plot debug data
if ~isempty(globalData.debug.p3p_outlier_keypoints) && plotParams.plot_p3p_outliers
    scatter(globalData.debug.p3p_outlier_keypoints(:,1), globalData.debug.p3p_outlier_keypoints(:,2), 5, 'red', 'Marker', 'x');
end
if ~isempty(globalData.debug.ckeypoints_invalid) && plotParams.plot_invalid_ckeypoints
    scatter(globalData.debug.ckeypoints_invalid(:,1), globalData.debug.ckeypoints_invalid(:,2), 5, 'magenta', 'Marker', 'o');
end
hold off; 
legend('candidate keypoints', 'keypoints', 'p3p outliers', 'invalid triangulations');  


end

