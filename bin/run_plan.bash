#!/bin/bash

# plan folder
docker exec -it pnp bash -ci "rosparam set /pnp_ros/plan_folder /home/robot/src/DIAG_demo/plans"

PLAN="DIAG_printer_1"

if [ "$1" != "" ]; then
  PLAN=$1
fi

if [ "$PLAN" != "stop" ]; then
  docker exec -it pnp bash -ci "cd ~/src/DIAG_demo/plans && pnpgen_translator inline $PLAN.plan"
fi

docker exec -it pnp bash -ci "rostopic pub /pnp/planToExec std_msgs/String \"data: '$PLAN'\" --once"

# to view the generated PNP
# docker exec -it pnp bash -ci "DISPLAY=:0 jarp.sh"

