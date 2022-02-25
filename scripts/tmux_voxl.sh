#!/bin/bash

#Check current date
CURR_YEAR=$(date "+%y")
CURR_DATE=$(date +%F-%H-%M-%S-%Z)
if [ $CURR_YEAR -eq 70 ]; then
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

#Check if sudo
tmux_sudo_suffix=''
if [ "$(whoami)" != "root" ]; then
  echo "Run script as sudo"
  tmux_sudo_suffix='sudo -s'
  exit 1
fi

#Stop cron
#echo "Stop cron job"
#sudo stop cron

#Restart snav
echo "Restarting snav"
systemctl stop snav
sleep 1s
systemctl start snav
sleep 5s

#Get confirmation from user if snav is restarted properly
# echo "Type "y" after snav is restarted to continue and press [Enter]"
# read entered_key
# if [[ ! "$entered_key" == "y" ]]; then
#   echo "Exiting script"
#   exit 1
# fi

SESSION_NAME=tmux_snav
if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

#MAV_TYPE=230
#MAV_MASS=0.245
#MAV_BOARD=sdf_tray
MAV_NAME=dragonfly$MAV_ID

tmux rename-window -t $SESSION_NAME "Ros"
tmux send-keys -t $SESSION_NAME "roscore" Enter

tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "sws; sleep 4; roslaunch snavquad_interface voxl_vio.launch pub_odom_base_link:=true imu_rate:=150" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sws; sleep 9; roslaunch snavquad_interface quad_control.launch use_vicon:=false" Enter

tmux new-window -t $SESSION_NAME -n "Cams"
tmux send-keys -t $SESSION_NAME "sws; roslaunch snavquad_interface stereo.launch board_type:=voxl"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sws; roslaunch snavquad_interface hires.launch board_type:=voxl"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sws; sleep 12; roslaunch snavquad_interface tof_with_tf.launch publish_depth_image:=true publish_ir_image:=false publish_point_cloud:=false"
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Aux"
tmux send-keys -t $SESSION_NAME "sws; sleep 12; roslaunch snavquad_interface snav_vio_overlay.launch" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sws; sleep 7; rosrun kr_trackers twist_to_velocity_goal.py __ns:=${MAV_NAME}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sws; roscd snavquad_interface/scripts/capture; chmod a+x record.sh; ./record.sh $MAV_ID"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sws; sleep 12; roslaunch kr_xbee_ros kr_xbee_ros_voxl.launch" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Dock"
tmux send-keys -t $SESSION_NAME "sws; sleep 12; roscd snavquad_interface/scripts; bash run_docker_tag.sh" Enter

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t "

tmux select-layout -t $SESSION_NAME tiled

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

clear
