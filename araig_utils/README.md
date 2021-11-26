# ARAIG Utils
This repo includes bash script for extracting images from bag files and generating videos.

## Dependencies
ros-package: image_view
> sudo apt-get install ros-${ROS_DISTRO}-image-view

ffmpeg for generating video from images
> sudo apt install ffmpeg

## Usage
1. Edit the saving path as **dir** in *export_video.sh*, bag file name and path to bag files.

2. Run the script
    >cd /path/to/catkin_ws/src/araig_test_stack/araig_utils
    >./launch/export_video.sh

3. Go to ${dir} to check the videos.

## TODO:
1. merge 2 launch files:
    
    2 launch files each for saving image for one camera, in which one bag file will be played once and the images from one topic will be saved and used for generating video.