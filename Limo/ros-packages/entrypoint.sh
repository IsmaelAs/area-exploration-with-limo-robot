#!/bin/bash

source /opt/ros/noetic/setup.bash
# source /agx_ws/devel/setup.bash
source /home/ubuntu/Documents/prj/devel/setup.bash


# copier les dossiers aux bons endroits
cp -r ./packages/launchs $(rospack find limo_bringup)
cp -r ./packages/params $(rospack find limo_bringup)

chmod a+x ./simulation/random-world-generator.sh

echo $(hostname -I)
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 5

if [ "$IS_SIMULATION" ];  then 

  # executer le random-world-generator
  ./simulation/random-world-generator.sh

  sleep 2m

  # Launch gmapping
  roslaunch  limo_gazebo_sim main_gmapping.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for gmapping to start up
  sleep 5

  # Launch map_merge
  roslaunch  limo_gazebo_sim map_merge.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for map_merge to start up
  sleep 5

  # Launch navigation stack
  roslaunch  limo_gazebo_sim main_navigation.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for navigation stack to start up
  sleep 10

  # Launch explore lite
  roslaunch  limo_gazebo_sim main_exploration.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for explore lite to start up
  sleep 5

  else 

  # Launch gmapping
  roslaunch  limo_bringup one_gmapping.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for gmapping to start up
  sleep 5

  # Launch map_merge
  roslaunch  limo_bringup map_merge.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for map_merge to start up
  sleep 5

  # Launch navigation stack
  roslaunch  limo_bringup one_navigation.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for navigation stack to start up
  sleep 5

  # Launch explore lite
  roslaunch  limo_bringup one_exploration.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

  # Wait for explore lite to start up


fi

# Subscribe to /exploration_state topic to control exploration state
# exec rosrun explore_control control_explore.py