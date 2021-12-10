# ARAIG Utils
This repo includes bash script for extracting images from bag files and generating videos.

## Dependencies
ros-package: image_view
> sudo apt-get install ros-${ROS_DISTRO}-image-view

ffmpeg for generating video from images
> sudo apt install ffmpeg

## Usage
1. Edit the variables:**BAGFILE**, **PATH2BAG**, **dir** and **rosws** at the beginning of *export_video.sh*:
- BAGFILE: filename of bag file
- PATH2BAG: path to the directory containing bag file
- dir: directory where to same frames and videos
- rosws: your ros workspace

2. Run the script *export_video.sh*

3. Go to ${dir} to check the videos.
