function updateCamTrajectoryPlot(viewId, globalData, ...
                                        trajActualPlot, trajEstimatedPlot, ...
                                        camPlot)

% Move the estimated camera in the plot
camPlot.Location = globalData.vSet.Views.Location{viewId};
camPlot.Orientation = globalData.vSet.Views.Orientation{viewId};

% Plot the estimated trajectory
locations = cat(1, globalData.vSet.Views.Location{:});
set(trajEstimatedPlot, 'XData', locations(:,1), 'YData', ...
    locations(:,2), 'ZData', locations(:,3));

% Plot the ground truth trajectory from actualVSet
locationsActual = cat(1, globalData.actualVSet.Views.Location{1:viewId});
set(trajActualPlot, 'XData', locationsActual(:,1), 'YData', ...
    locationsActual(:,2), 'ZData', locationsActual(:,3));

pcshow(globalData.landmarks,'black', 'MarkerSize', 10); 

% set axis limits 5 meters larger than data
limsx=get(gca,'XLim');
set(gca,'Xlim',[limsx(1)-5, limsx(2)+5]); 

limsy=get(gca,'YLim');
set(gca,'Ylim',[limsy(1)-5, limsy(2)+5]); 

limsz=get(gca,'ZLim');
set(gca,'Zlim',[limsz(1)-5, limsz(2)+5]); 