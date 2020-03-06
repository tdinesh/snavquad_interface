#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Input bag file location as argument"
  exit 1
fi

BAG_FILE=$1
if [! -f $BAG_FILE]; then
  echo "$BAG_FILE does not exist, please provide correct file"
  exit 1
fi

SESSION_NAME=tmux_bag_play

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

NET_IP=$(ifconfig | sed -En 's/127.0.0.1//;s/.*inet (addr:)?(([0-9]*\.){3}[0-9]*).*/\2/p')
MASTER_URI=http://localhost:11311


tmux rename-window -t $SESSION_NAME "Main"
tmux send-keys -t $SESSION_NAME "export ROS_IP=${NET_IP}; export ROS_MASTER_URI=${MASTER_URI}; rosrun rqt_mav_manager rqt_mav_manager" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "export ROS_IP=${NET_IP}; export ROS_MASTER_URI=${MASTER_URI}; roslaunch system_launch snav_rviz.launch mav_name:=${MAV_NAME}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "export ROS_IP=${NET_IP}; export ROS_MASTER_URI=${MASTER_URI}; ./record.sh ${MAV_ID}"
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t tmux_bag_play"

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME

clear