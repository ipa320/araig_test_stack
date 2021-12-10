#!/bin/bash
#BAGFILE=filename.bag
#PATH2BAG=/path/to/bagfile
#dir=path/to/directory
#rosws=path/to/ros_workspace

rosdir=~/.ros

mkdir -p $dir
cd $dir
rm frame*.jpg
source $rosws/devel/setup.bash

roslaunch araig_utils export.launch bagpath:=$PATH2BAG bag:=$BAGFILE
mv $rosdir/frame*.jpg $dir
cd $dir
ffmpeg -framerate 30 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt + ${PATH2BAG[i]}_cam1.mp4
rm frame*.jpg
rm $rosdir/frame*.ini

roslaunch araig_utils export2.launch bag:=${PATH2BAG[i]}/${BAG[i]}
mv $rosdir/frame*.jpg $dir
cd $dir
ffmpeg -framerate 30 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt + ${PATH2BAG[i]}_cam2.mp4
rm frame*.jpg
rm $rosdir/frame*.ini

