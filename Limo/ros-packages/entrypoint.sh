#!/bin/bash

source /opt/ros/noetic/setup.bash
source /agx_ws/devel/setup.bash

echo $(hostname -I)
roslaunch rosbridge_server rosbridge_websocket.launch &

if [ ! "$IS_SIMULATION" ];  then 
  # Launch gmapping
  rosrun gmapping slam_gmapping scan:=/scan 2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for gmapping to start up
  sleep 5

  # Launch navigation stack
  roslaunch limo_bringup limo_navigation_diff.launch 2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for navigation stack to start up
  sleep 5


  # Launch explore lite
  roslaunch explore_lite explore_costmap.launch scan:=/scan map:=/map 2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for explore lite to start up
  sleep 5
fi

# Subscribe to /exploration_state topic to control exploration state
exec rosrun explore_control control_explore.py