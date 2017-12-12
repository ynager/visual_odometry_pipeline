function globalData = applyScaleFactor(globalData)
% edit viewSet to compensate for unknown scale factor by looking at ground
% truth

camPoses = poses(globalData.vSet);
groundTruth = poses(globalData.actualVSet); 

% Move the first camera to the origin
locations = cat(1, camPoses.Location{:});
locations = locations - locations(1, :);

% Get locations of ground truth
locationsGT  = cat(1, groundTruth.Location{1:height(camPoses)});

% Get magnitudes of estimated and ground truth locations
magnitudes   = sqrt(sum(locations.^2, 2));
magnitudesGT = sqrt(sum(locationsGT.^2, 2));

% Calculate the scale factor
scaleFactor = median(magnitudesGT(2:end) ./ magnitudes(2:end));

% Scale the locations
locations = locations .* scaleFactor;
camPoses.Location = num2cell(locations, 2);

% Rotate the poses so that the first camera points along the Z-axis
R = camPoses.Orientation{1}';
for i = 1:height(camPoses)
    camPoses.Orientation{i} = camPoses.Orientation{i} * R;
end

globalData.vSet = updateView(globalData.vSet, camPoses);

end

