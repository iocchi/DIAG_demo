#!/bin/bash

SESSION=init

tmux -2 new-session -d -s $SESSION

tmux rename-window -t $SESSION:0 'goto'
tmux send-keys -t $SESSION:0 "cd ../actions && python goto_actionproxy.py" C-m

tmux new-window -t $SESSION:1 -n 'say'
tmux send-keys -t $SESSION:1 "cd ../actions && python say_actionproxy.py" C-m

tmux new-window -t $SESSION:2 -n 'lightcolor'
tmux send-keys -t $SESSION:2 "cd ../fluents && python lightcolor_fluentproxy.py" C-m

while [ 1 ]; do
  sleep 60
done

