#!/bin/bash

DCF=docker-compose.yml
XSERVER=""

export ROS_IP=127.0.0.1
export ROBOT_TYPE="stage"
export DEMO_DIR=`pwd | gawk '{ print gensub(/\/bin/, "", 1) }'`

if [ "$1" == "nvidia" ]; then
  DCF=docker-compose-nvidia.yml
fi
if [ "$1" == "dev" ]; then
  DCF=docker-compose-dev.yml
fi
if [ "$1" == "vnc" ]; then
  DCF=docker-compose-vnc.yml
  XSERVER=xserver
fi


if [ "$1" == "dev" ]; then
  # run docker services
  docker-compose -f $DCF up -d $XSERVER stage navigation speech vision \
    stagepersondetection actions pnp
else
  # pull docker services
  docker-compose -f $DCF pull $XSERVER stage navigation speech vision \
    stagepersondetection actions pnp

  # run docker services
  docker-compose -f $DCF up -d $XSERVER stage navigation speech vision \
    stagepersondetection actions pnp
fi

sleep 10

docker ps

sleep 3

# Stage with map
echo 'DISB1;marrtino' | netcat -w 1 localhost 9235
sleep 5




