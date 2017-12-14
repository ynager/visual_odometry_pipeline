function plotHandles = setupCamTrajectoryPlot(globalData)

%Setup Figure
%************************************************************************
figure();
set(gcf,'units','points','position',[300,300,800,400],'color','w');


%Setup PointCloud Axes
%************************************************************************
ax1 = axes('Position',[.05 .15 .4 .8]);
grid on
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal
rotate3d on;
hold on
view(0, 0); 

% Plot estimated camera pose
 camsize = 0.5;
 plotCamera('Size', camsize, 'Location',...
    globalData.vSet.Views.Location{1}, 'Orientation', globalData.vSet.Views.Orientation{1},...
    'Color', 'blue', 'Opacity', 0);

% Initialize camera trajectories
plot3(0, 0, 0, 'b-', 'LineWidth',2);
hold on; 
plot3(0, 0, 0, 'k:', 'LineWidth',2);
scatter3([], [], [],10,'black','filled','Marker','o');

legend('Estimated Trajectory', 'Ground Truth', 'Point Cloud', ...
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

