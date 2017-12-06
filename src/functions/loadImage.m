function image = load_image(ds, i)

    fprintf('\nLoading frame %d', i);
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
    
end