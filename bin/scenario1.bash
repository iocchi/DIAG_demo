#!/bin/bash

SESSION=scenario1

tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
  tmux -2 new-session -d -s $SESSION
  tmux rename-window -t $SESSION:0 'status'
  tmux new-window -t $SESSION:1 -n 'goto'
  tmux new-window -t $SESSION:2 -n 'say'
  tmux new-window -t $SESSION:3 -n 'pass'
  tmux new-window -t $SESSION:4 -n 'sense'
fi

echo "Configuring the environment ..."

tmux send-keys -t $SESSION:0 "./setrobotpose.bash 2 2 0 && ./setlight.bash red" C-m

echo "Starting ROS nodes ..."

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

echo "Starting action proxies ..."

docker exec -it actions bash -ci "cd ~/src/DIAG_demo/bin && ./actions_scenario1.bash"


# Continuous sensing not needed when sensing actions are used
#tmux new-window -t $SESSION:4 -n 'lightcolor'
#tmux send-keys -t $SESSION:4 "cd ../fluents && python lightcolor_fluentproxy.py" C-m

#tmux new-window -t $SESSION:5 -n 'open'
#tmux send-keys -t $SESSION:5 "cd ../fluents && python open_fluentproxy.py" C-m

echo "Done"

