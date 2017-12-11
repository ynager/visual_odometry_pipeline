function [trajActualPlot, trajEstimatedPlot, camPlot] = setupCamTrajectoryPlot(globalData)

figure();

% Set Y-axis to be vertical pointing down
view(gca, 3);
set(gca, 'CameraUpVector', [0, -1, 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);

grid on
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
hold on

camsize = 0.5;

% Plot estimated camera pose
camPlot = plotCamera('Size', camsize, 'Location',...
    globalData.vSet.Views.Location{1}, 'Orientation', globalData.vSet.Views.Orientation{1},...
    'Color', 'g', 'Opacity', 0);

% Initialize camera trajectories
trajEstimatedPlot = plot3(0, 0, 0, 'g-');
trajActualPlot    = plot3(0, 0, 0, 'b-');

legend('Estimated Trajectory', 'Ground Truth');
end

