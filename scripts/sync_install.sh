#!/bin/bash

if [[ $# -eq 0 ]] ; then
  echo 'Specify which robot to sync to'
else
  echo 'Syncing ws_indigo/install folder from computer to robot'
  rsync -r -a -v -e ssh --update --delete --exclude '*.git' --exclude '*.cmake' ~/voxl_home/ws_indigo/install root@dragonfly$1:/data/home_linaro/ws_indigo

  echo 'Syncing ws_noetic/install folder from computer to robot'
  rsync -r -a -v -e ssh --update --delete --exclude '*.git' --exclude '*.cmake' ~/voxl_home/ws_noetic/install root@dragonfly$1:/data/home_linaro/ws_noetic
fi
