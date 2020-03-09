#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Input mav number as argument"
  exit 1
fi

MAV_ID=$1
if echo $MAV_ID | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Calibrating camera for MAV $MAV_ID"
else
  echo "Input mav number(integer) as first argument"
  exit 1
fi

MAV_NAME=dragonfly${MAV_ID}

if [ $# -eq 1 ]; then
  echo "Input second argument as bagfile name"
  exit 1
fi

BAG_NAME=$2
if [ ! -f $BAG_NAME ]; then
  echo "$BAG_NAME does not exist, please provide correct file"
  exit 1
fi

SESSION_NAME=tmux_calibrate

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

tmux rename-window -t $SESSION_NAME "Main"
tmux send-keys -t $SESSION_NAME "export ROS_MASTER_URI=http://localhost:11311; roscore" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "export ROS_MASTER_URI=http://localhost:11311; sleep 2; rosbag play $BAG_NAME" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "export ROS_MASTER_URI=http://localhost:11311; sleep 2; rosrun camera_calibration cameracalibrator.py --size 10x7 --square 0.036 right:=$MAV_NAME/stereo/right/image_raw left:=$MAV_NAME/stereo/left/image_raw" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t tmux_calibrate"

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME

clear