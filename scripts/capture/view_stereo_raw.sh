#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Input mav number as argument"
  exit 1
fi

MAV_ID=$1
if echo $MAV_ID | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Visualize stereo for MAV $MAV_ID"
else
  echo "Input mav number(integer) as first argument"
  exit 1
fi

MAV_NAME=dragonfly${MAV_ID}
MAV_IP=192.168.0.${MAV_ID}
MAV_ADDRESS=${MAV_NAME}

if [ $# -eq 2 ]; then
  echo "Overriding mav IP $2"
  MAV_IP=$2
  MAV_ADDRESS=MAV_IP
fi

#NET_IP=$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')

NET_IP=$(ip route get 8.8.8.8 | head -1 | awk '{print $7}')
if [ -z "$NET_IP" ]; then
  NET_IP=$(ip -o -4 addr list wlan0 | awk '{print $4}' | cut -d/ -f1)
fi

MASTER_URI=http://${MAV_ADDRESS}:11311

SETUP_ROS_STRING="export ROS_IP=${NET_IP}; export ROS_MASTER_URI=${MASTER_URI}"

# if [ -z ${TMUX} ];
# then
#   TMUX= tmux new-session -s $SESSION_NAME -d
#   echo "Starting new session."
# else
#   echo "Already in tmux, leave it first."
#   exit
# fi

LAUNCH_PATH=$(rospack find snavquad_interface)
cp $LAUNCH_PATH/scripts/capture/rqt/StereoCalibrate.perspective ~/.ros
sed -i "s/dragonfly13/$MAV_NAME/g" ~/.ros/StereoCalibrate.perspective

$SETUP_ROS_STRING; rosrun rqt_gui rqt_gui --perspective-file ~/.ros/StereoCalibrate.perspective