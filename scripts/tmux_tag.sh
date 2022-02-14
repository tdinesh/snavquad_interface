#!/bin/bash

SESSION_NAME=tmux_tag

#tmux kill-session -t tmux_tag
#sleep 2s
ln -sf /opt/ptmx /dev/ptmx

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit  
fi

SETUP_ROS_STRING="source ~/home_linaro/.bashrc_voxl"

tmux setw -g mouse on

tmux rename-window -t $SESSION_NAME "Tag"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 7; roslaunch kr_multi_mav_manager multimaster_dragonfly_remote.launch old_api:=false" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 7; roslaunch tag_swarm tag_swarm.launch clamp_tag:=true origin_tag_id:=130 origin_tag_size:=0.96" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t "

#tmux select-layout -t $SESSION_NAME tiled

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME

clear
