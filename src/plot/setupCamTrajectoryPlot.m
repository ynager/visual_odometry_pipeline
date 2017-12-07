function [camActual, camEstimated, trajectoryActual, trajectoryEstimated] = setupCamTrajectoryPlot(vSet, groundTruth)

figure();

% Setup axes.
axis([-100, 100, -100, 100, -100, 100]);

% Set Y-axis to be vertical pointing down
view(gca, 3);
set(gca, 'CameraUpVector', [0, -1, 0]);
camorbit(gca, -120, 0, 'data', [0, 1, 0]);

grid on
xlabel('X (cm)');
ylabel('Y (cm)');
zlabel('Z (cm)');
hold on

camsize = 5;

% Plot estimated camera pose
camEstimated = plotCamera('Size', camsize, 'Location',...
    vSet.Views.Location{1}, 'Orientation', vSet.Views.Orientation{1},...
    'Color', 'g', 'Opacity', 0);

% Plot actual camera pose from groundTruth
camActual = plotCamera('Size', camsize, 'Location',...
    groundTruth.Views.Location{1}, 'Orientation', groundTruth.Views.Orientation{1},...
    'Color', 'b', 'Opacity', 0);

% Initialize camera trajectories
trajectoryEstimated = plot3(0, 0, 0, 'g-');
trajectoryActual    = plot3(0, 0, 0, 'b-');

legend('Estimated Trajectory', 'Ground Truth');
end

