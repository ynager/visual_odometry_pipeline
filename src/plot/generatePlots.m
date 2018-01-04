function generatePlots(varargin)

globalData = varargin{1}; 

if length(varargin) == 1
    sf = globalData.scale_factor;
else
    sf = varargin{2}; 
end

% Generate plots for report

%trajectory and point cloud plots
%************************************************************************
axes; 
grid minor
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal
rotate3d on;
hold on
view(0, 0); 


% Initialize camera trajectories
% Estimated Trajectory: 
locations = cat(1, globalData.vSet.Views.Location{:})*sf;
p1 = plot3(locations(:,1), locations(:,2), locations(:,3), 'b-', 'LineWidth',2);
hold on; 

% Actual Trajectory: 
locationsActual = cat(1, globalData.actualVSet.Views.Location{1:end});
p2 = plot3(locationsActual(:,1), locationsActual(:,2), locationsActual(:,3), 'k:', 'LineWidth',1.5);

% Total Landmarks: 
global_landmarks = globalData.landmarks*sf; 
p3 = scatter3(global_landmarks(:,1), global_landmarks(:,2), global_landmarks(:,3),5,'black','filled','Marker','o', 'MarkerFaceAlpha',.1,'MarkerEdgeAlpha',.1);
if size(locationsActual,1) > 1
    legend([p1, p2, p3], 'Estimated Trajectory', 'Actual Trajectory', 'Landmarks', 'Location', 'northeast');
else
    legend([p1, p3], 'Estimated Trajectory', 'Landmarks', 'Location', 'northeast');



view(0, 0);



end

