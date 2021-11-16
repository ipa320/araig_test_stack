#!/bin/bash
# variables for all bag files
BAG[0]=test3_2021-09-15-15-06-40.bag
BAG[1]=test3_2021-09-15-15-07-34.bag
BAG[2]=test3_2021-09-15-15-08-29.bag
BAG[3]=test3_2021-09-15-15-09-11.bag
BAG[4]=test3_2021-09-15-15-12-19.bag
BAG[5]=test3_2021-09-15-15-12-57.bag
BAG[6]=test3_2021-09-15-15-13-22.bag
BAG[7]=test3_2021-09-15-15-13-56.bag
BAG[8]=test3_2021-09-15-15-16-42.bag
BAG[9]=test3_2021-09-15-15-18-43.bag
BAG[10]=test3_2021-09-15-15-19-18.bag
BAG[11]=test3_2021-09-15-15-19-53.bag
BAG[12]=test3_2021-09-15-15-20-34.bag
BAG[13]=test3_2021-09-15-15-21-18.bag
BAG[14]=test3_2021-09-15-15-24-29.bag
BAG[15]=test3_2021-09-15-15-25-26.bag
BAG[16]=test3_2021-09-15-15-27-30.bag
BAG[17]=test3_2021-09-15-15-32-12.bag
BAG[18]=test3_2021-09-15-15-35-13.bag
BAG[19]=test3_2021-09-15-15-36-15.bag
# save all test names as array for rename the videos
PATH2BAG[0]=1_StraightToBulge
PATH2BAG[1]=2_DiagonalToBulge
PATH2BAG[2]=3_DiagonalToBulge
PATH2BAG[3]=4_diagnalToGap
PATH2BAG[4]=5_StraightToGap
PATH2BAG[5]=6_StraightToGap
PATH2BAG[6]=7_DiagonalToGap
PATH2BAG[7]=8_DiagonalToGap
PATH2BAG[8]=9_StraightToSmallGap
PATH2BAG[9]=10_StraightToSmallGap
PATH2BAG[10]=11_DiagonalToSmallGap
PATH2BAG[11]=12_DiagonalToSmallGap
PATH2BAG[12]=13_SlowlyStraightToSmallGap
PATH2BAG[13]=14_SlowlyStraightToSmallGap
PATH2BAG[14]=15_AlongGapOf2Textures
PATH2BAG[15]=16_DiagonalBetween2Textures
PATH2BAG[16]=17_OneWheelOnGap
PATH2BAG[17]=18_RampUp
PATH2BAG[18]=19_RampDown
PATH2BAG[19]=20_RampDown

# make directory to save all images and videos
dir=~/ARAIG/navel/test3/frames
rosdir=~/.ros

mkdir -p $dir
cd $dir
rm frame*.jpg
source ~/catkin_ws/devel/setup.bash

# loop for all tests
for i in 0
# 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
do
    roslaunch araig_utils export.launch bag:=${PATH2BAG[i]}/${BAG[i]}

    mv $rosdir/frame*.jpg $dir
    cd $dir
    # -i imput file url
    # -c codec      -c:[streamtype]     v for video
    # -pix_fmt set pixel format(resolution) + for same as input  
    ffmpeg -framerate 30 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt + ${PATH2BAG[i]}_cam1.mp4
    rm frame*.jpg
    # image_saver also save files indication info of cam, need to be deleted
    rm $rosdir/frame*.ini

    roslaunch araig_utils export2.launch bag:=${PATH2BAG[i]}/${BAG[i]}
    mv $rosdir/frame*.jpg $dir
    cd $dir
    ffmpeg -framerate 30 -i frame%04d.jpg -c:v libx264 -profile:v high -crf 20 -pix_fmt + ${PATH2BAG[i]}_cam2.mp4
    rm frame*.jpg
    rm $rosdir/frame*.ini
done
