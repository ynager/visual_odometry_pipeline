function undist_image = loadImage(ds, i, cameraParams)
    % load UNDISTORTED image
    
    % get img
    fprintf('\nLoading frame %d\n', i);
    if ds == 0
        image = imread(['../datasets/kitti/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        images = dir(['../datasets/malaga' ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        image = rgb2gray(imread(['../datasets/malaga/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread(sprintf('../datasets/parking/images/img_%05d.png',i))));
    elseif ds == 4
        image = im2uint8(rgb2gray(imread(sprintf('../datasets/alpstrasse/images/img_%05d.png',i))));
    elseif ds == 5
        image = im2uint8(rgb2gray(imread(sprintf('../datasets/alpstrasse/images/img_%05d.png',i))));
    else
        im = sprintf('/images/img_%05d.png',i); 
        loc = strcat('../datasets/', ds, im); 
        image = im2uint8(rgb2gray(imread(loc)));
    end
   
    % transform back to fulfil stupid matlab convention of K
    cp = cameraParameters('IntrinsicMatrix', cameraParams.IntrinsicMatrix', ...
                          'RadialDistortion', cameraParams.RadialDistortion, ...
                          'TangentialDistortion', cameraParams.TangentialDistortion); 
    undist_image = undistortImage(image, cp);
    
end