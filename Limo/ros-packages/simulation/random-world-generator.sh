#!/bin/bash



roslaunch limo_gazebo_sim random_world.launch random_world_number:=$((1 + $RANDOM % 6)) &

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




