function [scale_factor] = getScaleFactor(globalData, bootstrap_images)
% get the unknown scale factor from ground truth data

num_views = globalData.vSet.NumViews + (bootstrap_images(2) - bootstrap_images(1) - 1); 

camPoses = poses(globalData.vSet);
groundTruth = poses(globalData.actualVSet); 

locations = cat(1, camPoses.Location{:});

% Get locations of ground truth
locationsGT  = cat(1, groundTruth.Location{1:num_views});

% Delete left out steps of bootstrap
if(bootstrap_images(2)-bootstrap_images(1) > 1)
    locationsGT(bootstrap_images(1)+1:bootstrap_images(2)-1,:) = []; 
end

% Get magnitudes of estimated and ground truth locations
magnitudes   = sqrt(sum(locations.^2, 2));
magnitudesGT = sqrt(sum(locationsGT.^2, 2));

% Calculate the scale factor
scale_factor = median(magnitudesGT(2:end) ./ magnitudes(2:end));

end

