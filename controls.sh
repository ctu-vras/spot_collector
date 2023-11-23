#!/bin/bash
newsess="controls"

tmux new-session -d -s "$newsess" -n rviz
sleep 1
tmux send-keys -t "$newsess" "export ROS_MASTER_URI=http://spot-arm-1:11311/" Enter
tmux send-keys -t "$newsess" "source ~/catkin_ws/devel/setup.bash" Enter
tmux send-keys -t "$newsess" "rviz -d ~/catkin_ws/src/spot_collector/controls.rviz" Enter
tmux new-window -t "$newsess" -n interf
sleep 1
tmux send-keys -t "$newsess" "export ROS_MASTER_URI=http://spot-arm-1:11311/" Enter
tmux send-keys -t "$newsess" "source ~/catkin_ws/devel/setup.bash" Enter
tmux send-keys -t "$newsess" "rosrun spot_collector interface.py" Enter
tmux attach -t "$newsess"
