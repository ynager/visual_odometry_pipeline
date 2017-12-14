function undist_image = loadImage(ds, i, cameraParams)
    % load UNDISTORTED image
    
    % get img
    fprintf('\nLoading frame %d\n', i);
    if ds == 0
        image = imread(['../datasets/kitti/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread(['../datasets/malaga/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread(sprintf('../datasets/parking/images/img_%05d.png',i))));
    else
        assert(false);
    end
   
    % undist img
    undist_image = undistortImage(image, cameraParams);
    
end