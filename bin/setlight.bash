#!/bin/bash

# set light color status on simulator

LIGHTNAME=printer1light
LIGHTON=1
LIGHTCOLOR=red

if [ "$1" != "" ]; then
    LIGHTCOLOR=$1
fi 

docker exec -it stage bash -ci "rostopic pub /$LIGHTNAME/color std_msgs/String \"data: '$LIGHTCOLOR'\" --once  && rostopic pub /$LIGHTNAME/state std_msgs/Int8 \"data: $LIGHTON\" --once"

