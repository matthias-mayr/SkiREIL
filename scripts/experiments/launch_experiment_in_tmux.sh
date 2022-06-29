#!/bin/sh
#
# Setup an experiment in tmux
# It assume an existing tmux session

# Left columns: roscore, robot_bringup, skiros.launch
# roscore
tmux send-keys "roscore" C-m
sleep 10

tmux splitw -h -p 60
tmux selectp -t 0
tmux splitw -v -p 80
tmux send-keys "roscd skireil && scripts/build/prepare_ros.sh" C-m
tmux send-keys "mon launch skireil robot_bringup.launch" C-m
# Give robot_bringup.launch some time
sleep 3

tmux splitw -v -p 70
tmux send-keys "mon launch skireil skiros.launch" C-m

# Right column: htop and Black-DROPS
tmux selectp -t 3
tmux send-keys "htop" C-m
# Give skiros.launch some time
sleep 8

tmux splitw -v -p 70
tmux send-keys "roscd skireil; scripts/build/configure.sh; scripts/build/configure.sh" C-m
# Not sent yet, but ready to be started
tmux send-keys "scripts/experiments/run_learning.sh 1"
