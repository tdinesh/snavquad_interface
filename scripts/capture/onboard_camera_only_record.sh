#!/bin/sh

if [ $# -eq 0 ]; then
  echo "Input vehicle number as first argument"
  exit 1
fi

MAV_ID=$1
if echo $MAV_ID | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Recording bag file for MAV $MAV_ID"
else
  echo "Input mav number(integer) as first argument"
  exit 1
fi

MAV_NAME=dragonfly${MAV_ID}
echo "MAV Name $MAV_NAME"

bag_folder="/media/sdcard"

if [ ! -d "$bag_folder" ]; then
  echo "*** WARNING *** SD card not present, recording locally"
  cd ~/
else
  echo 'Bag files stored at' $bag_folder
  cd $bag_folder

  #Print out %Used SD card
  USED_PERCENT=$(df --output=pcent $bag_folder | awk '/[0-9]%/{print $(NF)}' | awk '{print substr($1, 1, length($1)-1)}')
  echo 'SD card' ${USED_PERCENT} '% Full'
fi

TOPICS="
/tf
/$MAV_NAME/imu
"

DFC_TOPICS="
/$MAV_NAME/dfc/fisheye/camera_info
/$MAV_NAME/dfc/camera_info
/$MAV_NAME/dfc/image_raw"

HIGHRES_TOPICS="
/$MAV_NAME/highres/camera_info
/$MAV_NAME/highres/image_raw"

STEREO_TOPICS="
/$MAV_NAME/stereo/left/image_raw
/$MAV_NAME/stereo/left/camera_info
/$MAV_NAME/stereo/right/image_raw
/$MAV_NAME/stereo/right/camera_info
"

ALL_TOPICS=$TOPICS$DFC_TOPICS$HIGHRES_TOPICS$STEREO_TOPICS

BAG_STAMP=$(date +%F-%H-%M-%S-%Z)
CURR_TIMEZONE=$(date +%Z)

BAG_NAME=$BAG_STAMP-V${MAV_ID}.bag
BAG_PREFIX=V${MAV_ID}-${CURR_TIMEZONE}-calib

eval rosbag record -b512 $ALL_TOPICS -o $BAG_PREFIX