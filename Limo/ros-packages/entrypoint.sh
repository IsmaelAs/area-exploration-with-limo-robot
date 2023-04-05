#!/bin/bash

source /opt/ros/noetic/setup.bash


echo $(hostname -I)
sleep 5
source /agx_ws/devel/setup.bash

if [ ! "$IS_SIMULATION" ];  then 

  roslaunch rosbridge_server rosbridge_websocket.launch &

  cp -r ./packages/launchs $(rospack find limo_bringup)
  cp -r ./packages/params $(rospack find limo_bringup)

  # Launch gmapping
  roslaunch --wait  limo_bringup one_gmapping.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for gmapping to start up
  sleep 5

  # Launch map_merge
  roslaunch --wait  limo_bringup map_merge.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for map_merge to start up
  sleep 5

  # Launch navigation stack
  roslaunch  --wait  limo_bringup one_navigation.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for navigation stack to start up
  sleep 5

  rosrun explore_control control_explore.py &

  # Wait for explore lite to start up
  sleep 10

  # Launch explore lite
  exec roslaunch --wait  limo_bringup one_exploration.launch  2> >(grep -v TF_REPEATED_DATA buffer_core)

else 
  exec rosrun explore_control control_explore.py

fi

# Subscribe to /exploration_state topic to control exploration state