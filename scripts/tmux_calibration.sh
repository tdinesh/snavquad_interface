#!/bin/bash

#Check current date
CURR_YEAR=$(date "+%y")
CURR_DATE=$(date +%F-%H-%M-%S-%Z)
if [ $CURR_YEAR -eq 69 ]; then
  echo "Please update current date $CURR_DATE before proceeding"
  exit 1
fi

#Check if MAV_ID is set
if echo $MAV_ID | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Running system for MAV $MAV_ID"
else
  echo "Please set MAV_ID variable in bashrc"
  exit 1
fi

MAV_TYPE=dragonfly

#Check if sudo
tmux_sudo_suffix=''
if [ "$(whoami)" != "root" ]; then
  echo "Run script as sudo"
  tmux_sudo_suffix='sudo -s'
  exit 1
fi

#Stop cron
echo "Stop cron job"
sudo stop cron
#Restart snav
echo "Restarting snav"
sudo stop snav
sleep 1s
sudo start snav
sleep 1s

#Get confirmation from user if snav is restarted properly
echo "Type "y" after snav is restarted to continue and press [Enter]"
read entered_key
if [[ ! "$entered_key" == "y" ]]; then
  echo "Exiting script"
  exit 1
fi

SESSION_NAME=tmux_tag
if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

USE_DFS=false

tmux rename-window -t $SESSION_NAME "Ros"
tmux send-keys -t $SESSION_NAME "roscore" Enter

tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "sleep 4; roslaunch tag_swarm vio_qc.launch mav_type:=${MAV_TYPE} mass:=0.240 show_down_img:=true down_img_pub_rate:=10 control_rate:=100" Enter
# tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "sleep 7; roslaunch system_launch disp_nodelet.launch use_gpu:=false use_dfs:=$USE_DFS exposure:=1.0 gain:=0.3" Enter

tmux new-window -t $SESSION_NAME -n "Sub"
tmux send-keys -t $SESSION_NAME "sleep 7; roslaunch tag_swarm tag_swarm.launch origin_tag_id:=123 origin_tag_size:=0.2" Enter

tmux new-window -t $SESSION_NAME -n "stereo"
tmux send-keys -t $SESSION_NAME "sleep 7; roslaunch snavquad_interface stereo.launch exposure:=0.3 width:=640 height:=480" Enter
# tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "sleep 7; roslaunch system_launch snav_traj_replanning.launch use_dfs:=$USE_DFS" Enter
# tmux split-window -t $SESSION_NAME
# tmux send-keys -t $SESSION_NAME "sleep 7; roslaunch system_launch jps3d.launch" Enter


tmux new-window -t $SESSION_NAME -n "Bagging"
tmux send-keys -t $SESSION_NAME "roslaunch snavquad_interface calibration_bagging.launch"
# tmux send-keys -t $SESSION_NAME "roslaunch system_launch fsm_global_path.launch sm_type:=global do_loop:=true"
# tmux split-window -t $SESSION_NAME -h
# tmux send-keys -t $SESSION_NAME "sleep 1; htop"

# tmux new-window -t $SESSION_NAME -n "Bag"
# tmux send-keys -t $SESSION_NAME "./onboard_camera_only_record.sh $MAV_ID"

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t "

tmux select-layout -t $SESSION_NAME tiled

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

clear
