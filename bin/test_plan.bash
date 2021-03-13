#!/bin/bash

# plan folder
docker exec -it pnp bash -ci "rosparam set /pnp_ros/plan_folder /home/robot/src/DIAG_demo/plans"


if [ "$1" == "stop" ]; then

docker exec -it pnp bash -ci "rostopic pub /pnp/planToExec std_msgs/String \"data: 'stop'\" --once"

else

docker exec -it pnp bash -ci "cd ~/src/DIAG_demo/plans && pnpgen_translator inline DIAG_printer.plan"

docker exec -it pnp bash -ci "rostopic pub /pnp/planToExec std_msgs/String \"data: 'DIAG_printer'\" --once"

fi

# to view the generated PNP
#docker exec -it pnp bash -ci "DISPLAY=:0 jarp.sh"



