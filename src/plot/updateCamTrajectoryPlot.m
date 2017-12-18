function updateCamTrajectoryPlot(viewId, globalData, currState, I, plotHandles)

% Move the estimated camera in the plot
h = plotHandles.axes1.Children(5);
M = cat(1,globalData.vSet.Views.Orientation{viewId}*0.5, zeros(1,3)); 
M = cat(2,M,[globalData.vSet.Views.Location{viewId}, 1]'); 
h.Matrix = M; 

% Plot the estimated trajectory
h = plotHandles.axes1.Children(4);
locations = cat(1, globalData.vSet.Views.Location{:});
set(h, 'XData', locations(:,1), 'YData', ...
    locations(:,2), 'ZData', locations(:,3));

% Plot the ground truth trajectory from actualVSet
h = plotHandles.axes1.Children(3);
locationsActual = cat(1, globalData.actualVSet.Views.Location{1:viewId});
set(h, 'XData', locationsActual(:,1), 'YData', ...
    locationsActual(:,2), 'ZData', locationsActual(:,3));

% Updata point cloud
h = plotHandles.axes1.Children(2);
set(h, 'XData', globalData.landmarks(:,1), 'YData', ...
    globalData.landmarks(:,2), 'ZData', globalData.landmarks(:,3));

% Updata inlier point cloud
h = plotHandles.axes1.Children(1);
set(h, 'XData', currState.landmarks(:,1), 'YData', ...
    currState.landmarks(:,2), 'ZData', currState.landmarks(:,3));
 

% set axis limits 5 meters larger than data
limsx=get(plotHandles.axes1,'XLim');
set(plotHandles.axes1,'Xlim',[limsx(1), limsx(2)]); 

limsy=get(plotHandles.axes1,'YLim');
set(plotHandles.axes1,'Ylim',[limsy(1), limsy(2)]); 

limsz=get(plotHandles.axes1,'ZLim');
set(plotHandles.axes1,'Zlim',[limsz(1), limsz(2)]); 

axes(plotHandles.axes2);   
imshow(I); 
num_inliers = length(globalData.vSet.Views.Points{viewId}); 
title(['Number of inliers: ',num2str(num_inliers)]);
hold on; 

%draw points
scatter(currState.candidate_kp(:,1), currState.candidate_kp(:,2), 5, 'blue', 'filled', 'Marker', 'o'); 
scatter(currState.keypoints(:,1), currState.keypoints(:,2), 5, 'green', 'filled', 'Marker', 'o'); 
hold off; 

%draw displacement vectors
%for i = 1:num_inliers
%    x_from = globalData.vSet.Views.Points{viewId-1}(i,1); 
%    x_to = globalData.vSet.Views.Points{viewId}(i,1);
%    y_from = globalData.vSet.Views.Points{viewId-1}(i,2);
%    y_to = globalData.vSet.Views.Points{viewId}(i,2);
    
%    plot([x_from; x_to], [y_from; y_to], 'r-', 'Linewidth', 0.2);
%    hold on; 
%end



