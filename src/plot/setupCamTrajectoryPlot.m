function plotHandles = setupCamTrajectoryPlot(globalData)

close all;
run parameters.m

%Setup Figure
%************************************************************************
figure();
set(gcf,'units','points','position',[400,100,900,400],'color','w');

%Setup Video Capture
%************************************************************************
if plotParams.record_video
    mkdir media; 
    plotHandles.writerObj = VideoWriter('media/vis_od.mp4'); 
    plotHandles.writerObj.FrameRate = plotParams.video_framerate;
    plotHandles.writerObj.Quality = 70; 
    open(plotHandles.writerObj);
    disp('Video Recording: On'); 
end

%Setup PointCloud Axes
%************************************************************************
ax1 = axes('Position',[.05 .10 .4 .8]);
grid minor
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal
rotate3d on;
hold on
view(0, 0); 

% Get estimated camera location and orientation
loc = globalData.vSet.Views.Location{1}; 
orient = globalData.vSet.Views.Orientation{1};
heading = orient * [0, 0, 1]'; 
heading = 10*heading/norm(heading);
camh = heading + loc'; 

% Current Location: 
scatter3(loc(1), loc(2), loc(3), 40, 'red','filled', 'Marker','o');
hold on; 
% Current Heading: 
plot3([loc(1), camh(1)], [loc(2), camh(2)], [loc(3), camh(3)], 'red', 'LineWidth', 1.5); 

% Initialize camera trajectories
% Estimated Trajectory: 
p3 = plot3(0, 0, 0, 'b-', 'LineWidth',2);
% Actual Trajectory: 
locationsActual = cat(1, globalData.actualVSet.Views.Location{1:end});
p4 = plot3(locationsActual(:,1), locationsActual(:,2), locationsActual(:,3), 'k:', 'LineWidth',1.5);
% Total Landmarks: 
p5 = scatter3([], [], [],5,'black','filled','Marker','o', 'MarkerFaceAlpha',.1,'MarkerEdgeAlpha',.1);
% Current Landmarks: 
p6 = scatter3([], [], [],10,'green','filled','Marker','o', 'MarkerFaceAlpha',.5,'MarkerEdgeAlpha',.5);

legend([p3, p4, p5, p6], 'Estimated Trajectory', 'Actual Trajectory', ...
    'Total landmarks', 'currState landmarks', 'Location','northeast');
view(0, 0);


% Setup Image Axes
% ***********************************************************************
ax2 = axes('Position',[.45 .2 .55 .55]); 
plotHandles.I = imshow([]); 


% Populate plotHandles
plotHandles.axes1 = ax1; 
plotHandles.axes2 = ax2;
end

