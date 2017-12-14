function [ bearings ] = getBearingVector( keypoints, K )
%getBearingVector takes in keypoints and calculate bearing vectors, which
%is the unit vector of the keypoints in the Camera frame
%(vector pointing on UnitImagePlane but normalized)
%
%   keypoints:  Mx2 Matrix: [u v; ...]
%   K: camera Matrix
%
%   bearings: Mx3 Matirx: [x y z; ...]

xy1 = K\[keypoints';ones(1,size(keypoints,1))];
xy1 = normc(xy1); % normalize columns

bearings = xy1';

end

