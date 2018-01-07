# Vision Algorithms for Mobile Robotics #
## Visual Odometry Pipeline ##

The goal of this mini-project is to implement a simple, monocular, visual odometry (VO) pipeline 
with the most essential features: initialization of 3D landmarks, keypoint tracking between two 
frames, pose estimation using established 2D ↔ 3D correspondences, and triangulation of new 
landmarks.

### Prerequisites ###
MATLAB 2017b with

- Image Toolbox

- Optimization Toolbox

- Statistics Toolbox

### How To Run The Pipeline ###

1.) Clone the repository.

2.) Download the standard datasets (KITTI, Malaga, Parking) from 
    http://rpg.ifi.uzh.ch/teaching.html and add copy them into the
    visual_odometry_project/datasets directory. The directories must be renamed to 
    'kitti', 'malaga', and 'parking'. 

3.) in src/parameters.m, select the dataset by setting the variable ds. 
    Optionally, also change the pipeline parameters there

4.) go into visual_odometry_project/src, and run main.m to start the pipeline.

### Screencasts ###

Please find the link to the screencasts of this pipeline applied to six different 
datasets below:

https://drive.google.com/open?id=1AHjNfthDAVzBY2az-X1W9HUt4fE05oeL

