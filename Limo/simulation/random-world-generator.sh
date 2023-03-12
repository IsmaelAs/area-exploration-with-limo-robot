#!/bin/bash


cp -r launch $(rospack find limo_gazebo_sim)
cp -r worlds $(rospack find limo_gazebo_sim)

roslaunch limo_gazebo_sim random_world.launch random_world_number:=$((1 + $RANDOM % 3))
