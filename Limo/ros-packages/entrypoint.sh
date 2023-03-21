#!/bin/bash

source /opt/ros/noetic/setup.bash
echo $(hostname -I)
roslaunch rosbridge_server rosbridge_websocket.launch &

# Launch gmapping
rosrun gmapping slam_gmapping scan:=/limo/scan 2> >(grep -v TF_REPEATED_DATA buffer_core) &

# Wait for gmapping to start up
sleep 5

# Launch navigation stack
roslaunch limo_bringup limo_navigation_diff.launch 2> >(grep -v TF_REPEATED_DATA buffer_core) &

# Wait for navigation stack to start up
sleep 5


# Launch explore lite
roslaunch explore_lite explore_costmap.launch 2> >(grep -v TF_REPEATED_DATA buffer_core) &

# Wait for explore lite to start up
sleep 5


# Publish /exploration_state topic 
rosrun explore_control publish_explore.py &

# Subscribe to /exploration_state topic to control exploration state
exec rosrun explore_control control_explore.py