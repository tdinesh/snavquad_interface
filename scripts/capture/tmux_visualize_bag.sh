#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Input mav number as argument"
  exit 1
fi

MAV_ID=$1
if echo $MAV_ID | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Visualize bag for MAV $MAV_ID"
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

SESSION_NAME=tmux_vis

#Generate rviz config file for specific mav from default one
RVIZ_CONFIG_FILE="$(realpath ~/.ros/$MAV_NAME.rviz)"
LAUNCH_PATH=$(rospack find system_launch)
cp $LAUNCH_PATH/config/rviz/dragonfly_gs2.rviz ${RVIZ_CONFIG_FILE}
sed -i "s/dragonfly18/$MAV_NAME/g" ${RVIZ_CONFIG_FILE}

CAMS_PERSP=CamsLRBMax.perspective
cp $LAUNCH_PATH/config/rqt/${CAMS_PERSP} ~/.ros
sed -i "s/dragonfly18/$MAV_NAME/g" ~/.ros/${CAMS_PERSP}

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

tmux rename-window -t $SESSION_NAME "Roscore"
tmux send-keys -t $SESSION_NAME "export ROS_MASTER_URI=http://localhost:11311; roscore" Enter

tmux new-window -t $SESSION_NAME -n "Main"
tmux send-keys -t $SESSION_NAME "export ROS_MASTER_URI=http://localhost:11311; sleep 2; rosbag play $BAG_NAME"
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "export ROS_MASTER_URI=http://localhost:11311; sleep 3; rosrun rqt_gui rqt_gui --perspective-file ~/.ros/${CAMS_PERSP}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "export ROS_MASTER_URI=http://localhost:11311; sleep 3; roslaunch system_launch snav_rviz.launch mav_name:=$MAV_NAME rviz_config:=$RVIZ_CONFIG_FILE" Enter
tmux select-layout -t $SESSION_NAME tiled
#tmux split-window -t $SESSION_NAME
#tmux send-keys -t $SESSION_NAME "export ROS_MASTER_URI=http://localhost:11311; sleep 3; roslaunch system_launch snav_tf_pub.launch mav_name:=$MAV_NAME" Enter
#tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t tmux_vis"

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME

clear