#!/bin/bash

SESSION_NAME=tmux_calib

#Check if sudo
tmux_sudo_suffix=''
if [ "$(whoami)" != "root" ]; then
  echo "Run script as sudo"
  tmux_sudo_suffix='sudo -s'
  exit 1
fi

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

QVGA=1

if [ "$QVGA" -eq 1 ];
then
  WIDTH=320
  HEIGHT=240
else
  WIDTH=640
  HEIGHT=480
fi

tmux rename-window -t $SESSION_NAME "Roscore"
tmux send-keys -t $SESSION_NAME "roscore" Enter

tmux new-window -t $SESSION_NAME -n "Calib"
tmux send-keys -t $SESSION_NAME "sleep 4; roslaunch snavquad_interface stereo.launch exposure:=0.3 width:=$WIDTH height:=$HEIGHT" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 5; rosbag record -b 512 -o calib /$MAV_NAME/stereo/left/image_raw /$MAV_NAME/stereo/right/image_raw"

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t tmux_calib"

tmux select-layout -t $SESSION_NAME tiled

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

clear
