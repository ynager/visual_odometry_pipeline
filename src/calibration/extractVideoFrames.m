function extractVideoFrames(video_path, images_dir, n, scale)
% Extracts every n'th frame from a video at 'video_path' and saves 
% them in 'images_dir' named with the given 'image_prefix'
% before saving, images are resized by a scaling factor 'scale'

% check if stuff exists
if(~exist(video_path, 'file'))
    error('Specified video file does not exist! '); 
    return
end
if(~exist(images_dir, 'dir'))
    fprintf('\nCreating directory: %s\n', images_dir);  
    mkdir(images_dir); 
end


% process the video 
vid=VideoReader(video_path);

iFrame = 0;
nFrame = 0; 
fprintf('Extracting'); 
while hasFrame(vid)
  frames = readFrame(vid);
  
  if mod(iFrame, n) == 0
    frames = imresize(frames,scale); 
    imwrite(frames, fullfile(images_dir, sprintf('img_%05d.png', nFrame)));
    nFrame = nFrame + 1;
    
    if mod(nFrame,5) == 0 
     fprintf('.'); 
     fig = imshow(frames);
     pause(0.01); 
    end
    
  end
  
  iFrame = iFrame + 1;
  
end 
fprintf('\nSuccesfully saved %d frames to %s\n\n',[nFrame, images_dir]);
close(fig); 
end

