function plotHandles = setupCamTrajectoryPlot(globalData)

close all;

%Setup Figure
%************************************************************************
figure();
set(gcf,'units','points','position',[300,100,800,400],'color','w');


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

% Plot estimated camera location and orientation
loc = globalData.vSet.Views.Location{1}; 
orient = globalData.vSet.Views.Orientation{1};
heading = orient * [0, 0, 1]'; 
heading = 10*heading/norm(heading);
camh = heading + loc'; 

scatter3(loc(1), loc(2), loc(3), 40, 'black', 'Marker','o');
hold on; 
plot3([loc(1), camh(1)], [loc(2), camh(2)], [loc(3), camh(3)], 'black', 'LineWidth', 1.2); 

% Initialize camera trajectories
p3 = plot3(0, 0, 0, 'b-', 'LineWidth',2);
p4 = plot3(0, 0, 0, 'k:', 'LineWidth',2);
p5 = scatter3([], [], [],5,'black','filled','Marker','o');
p6 = scatter3([], [], [],10,'green','filled','Marker','o');

legend([p3, p4, p5, p6], 'Estimated Trajectory', 'Ground Truth', 'Total landmarks', 'currState landmarks', ...
    'location','north');
view(0, 0);


% Setup Image Axes
% ***********************************************************************
ax2 = axes('Position',[.4 .2 .65 .65]); 
plotHandles.I = imshow([]); 


% Populate plotHandles
plotHandles.axes1 = ax1; 
plotHandles.axes2 = ax2;
end

