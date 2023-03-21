#!/bin/bash

source /opt/ros/noetic/setup.bash
echo $(hostname -I)
xterm -e "roslaunch rosbridge_server rosbridge_websocket.launch" &

# chmod explore_control files
exec chmod +x /catkin_ws/src/explore_control/publish_explore.py
exec chmod +x /catkin_ws/src/explore_control/control_explore.py

# Launch gmapping
xterm -e "rosrun gmapping slam_gmapping scan:=/limo/scan 2> >(grep -v TF_REPEATED_DATA buffer_core)" &

# Wait for gmapping to start up
sleep 5

# Launch navigation stack
xterm -e "roslaunch limo_bringup limo_navigation_diff.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)" &

# Wait for navigation stack to start up
sleep 5


# Launch explore lite
xterm -e "roslaunch explore_lite explore_costmap.launch 2> >(grep -v TF_REPEATED_DATA buffer_core)" &

# Wait for explore lite to start up
sleep 5


# Publish /exploration_state topic 
rosrun explore_control publish_explore.py &

# Subscribe to /exploration_state topic to control exploration state
rosrun explore_control control_explore.py &