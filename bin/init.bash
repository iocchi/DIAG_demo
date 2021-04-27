#!/bin/bash

SESSION=init

tmux -2 new-session -d -s $SESSION

tmux rename-window -t $SESSION:0 'goto'
tmux send-keys -t $SESSION:0 "cd ../actions && python goto_actionproxy.py" C-m

tmux new-window -t $SESSION:1 -n 'say'
tmux send-keys -t $SESSION:1 "cd ../actions && python say_actionproxy.py" C-m

tmux new-window -t $SESSION:2 -n 'pass'
tmux send-keys -t $SESSION:2 "cd ../actions && python pass_actionproxy.py" C-m

tmux new-window -t $SESSION:3 -n 'sense'
tmux send-keys -t $SESSION:3 "cd ../actions && python sense_actionproxy.py" C-m

tmux new-window -t $SESSION:4 -n 'getimage'
tmux send-keys -t $SESSION:4 "cd ../actions && python getimage_actionproxy.py" C-m


# Continuous sensing not needed when sensing actions are used
#tmux new-window -t $SESSION:4 -n 'lightcolor'
#tmux send-keys -t $SESSION:4 "cd ../fluents && python lightcolor_fluentproxy.py" C-m

#tmux new-window -t $SESSION:5 -n 'open'
#tmux send-keys -t $SESSION:5 "cd ../fluents && python open_fluentproxy.py" C-m

while [ 1 ]; do
  sleep 60
done

