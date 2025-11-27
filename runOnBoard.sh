#!/bin/bash

#tmux kill-ser

PROJECT_PATH="/home/pi/RL"

function new_3_win {
tmux split-window -v
tmux select-pane -t 0
tmux split-window -h
tmux select-pane -t 2
}

function run { #pane number, path, script
tmux select-pane -t $1
tmux send-keys "cd $PROJECT_PATH/$2" ENTER
tmux send-keys "python3 $3" ENTER
}

tmux new-session -d -s runRL
sleep 1

new_3_win

tmux select-pane -t 0
sleep 1
tmux send-keys "conda activate RL" ENTER
sleep 3
tmux send-keys "cd $PROJECT_PATH/src && python hardware_adapter.py" ENTER

tmux select-pane -t 1
sleep 1
tmux send-keys "conda activate RL" ENTER
sleep 3
tmux send-keys "cd $PROJECT_PATH/src && python system_manager.py" ENTER


#run 2 src "quadCmdGate.py"
