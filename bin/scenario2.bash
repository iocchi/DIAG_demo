#!/bin/bash

# host system

SESSION=scenario2

tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then
  tmux -2 new-session -d -s $SESSION
  tmux rename-window -t $SESSION:0 'status'
fi

echo "Configuring the environment ..."

tmux send-keys -t $SESSION:0 "./setrobotpose.bash 0.85 1.75 0 && ./setpersonpose.bash alice 1.75 1.75 180" C-m

echo "Starting ROS nodes ..."

# Navigation
echo '@loc' | netcat -w 1 localhost 9238
sleep 3
echo '@movebasegbn' | netcat -w 1 localhost 9238
sleep 3

# Speech

echo '@audio' | netcat -w 1 localhost 9239
sleep 3

 
# check
docker exec -it stage bash -ci "rosnode list"


echo "Starting action proxies ..."

docker exec -it actions bash -ci "cd ~/src/DIAG_demo/bin && ./actions_scenario2.bash"

echo "Done"

