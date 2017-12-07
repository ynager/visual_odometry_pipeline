function updateCamTrajectoryPlot(viewId, posesEstimated, posesActual, ...
                                        trajEstimated, trajActual, ...
                                        camEstimated, camActual)

% Move the estimated camera in the plot
camEstimated.Location = posesEstimated.Location{viewId};
camEstimated.Orientation = posesEstimated.Orientation{viewId};

% Move the actual camera in the plot
camActual.Location = posesActual.Location{viewId};
camActual.Orientation = posesEstimated.Orientation{viewId};

% Plot the estimated trajectory
locations = cat(1, posesEstimated.Location{:});
set(trajEstimated, 'XData', locations(:,1), 'YData', ...
    locations(:,2), 'ZData', locations(:,3));

% Plot the ground truth trajectory
locationsActual = cat(1, posesActual.Location{1:viewId});
set(trajActual, 'XData', locationsActual(:,1), 'YData', ...
    locationsActual(:,2), 'ZData', locationsActual(:,3));