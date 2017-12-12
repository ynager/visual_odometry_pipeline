function [trajActualPlot, trajEstimatedPlot, camPlot] = setupCamTrajectoryPlot(globalData)

figure();

grid on
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal
rotate3d on;
hold on
view(0, 0); 

camsize = 0.5;

% Plot estimated camera pose
camPlot = plotCamera('Size', camsize, 'Location',...
    globalData.vSet.Views.Location{1}, 'Orientation', globalData.vSet.Views.Orientation{1},...
    'Color', 'g', 'Opacity', 0);

% Initialize camera trajectories
trajEstimatedPlot = plot3(0, 0, 0, 'g-');
trajActualPlot    = plot3(0, 0, 0, 'b-');
pcshow(globalData.landmarks,'black', 'MarkerSize', 10);

legend('Estimated Trajectory', 'Ground Truth', 'Point Cloud');

end

