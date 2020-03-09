#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Input mav number as argument"
  exit 1
fi

MAV_ID=$1
if echo $MAV_ID | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Setting date for MAV $MAV_ID"
else
  echo "Input mav number(integer) as first argument"
  exit 1
fi

MAV_USER_NAME=linaro
MAV_IP=192.168.131.${MAV_ID}
MAV_ADDRESS=dragonfly${MAV_ID}

if [ $# -eq 2 ]; then
  echo "Overriding mav IP $2"
  MAV_IP=$2
  MAV_ADDRESS=MAV_IP
fi

CURR_DATE="$(date "+%e %b %G %H:%M:%S")"
TIMEZONE="$(cat /etc/timezone)"
echo $CURR_DATE

ssh -t ${MAV_USER_NAME}@${MAV_ADDRESS} "sudo timedatectl set-timezone '${TIMEZONE}'"
ssh -t ${MAV_USER_NAME}@${MAV_ADDRESS} "sudo date --set '${CURR_DATE}'"