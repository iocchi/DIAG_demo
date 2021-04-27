#!/bin/bash

DCF=docker-compose.yml
XSERVER=""

export ROS_IP=127.0.0.1
export ROBOT_TYPE="stage"
export DEMO_DIR=`pwd | gawk '{ print gensub(/\/bin/, "", 1) }'`

if [ "$1" == "dev" ]; then
  DCF=docker-compose-dev.yml
fi
if [ "$1" == "vnc" ]; then
  DCF=docker-compose-vnc.yml
  XSERVER=xserver
fi


if [ "$1" == "dev" ]; then
  # run docker services
  docker-compose -f $DCF up -d $XSERVER stage navigation actions speech vision pnp
else
  # pull docker services
  docker-compose -f $DCF pull $XSERVER stage navigation actions speech pnp

  # run docker services
  docker-compose -f $DCF up -d $XSERVER stage navigation actions speech pnp
fi

sleep 5

docker ps

sleep 1

# Stage with map
echo 'DISB1;marrtino' | netcat -w 1 localhost 9235
sleep 5

# Navigation
echo '@loc' | netcat -w 1 localhost 9238
sleep 3
echo '@movebasegbn' | netcat -w 1 localhost 9238
sleep 3

# Speech

echo '@audio' | netcat -w 1 localhost 9239
sleep 3

# Vision  (use marrtino as robot in stage)

echo '@takephoto' | netcat -w 1 localhost 9237
sleep 3


# check
docker exec -it stage bash -ci "rosnode list"


