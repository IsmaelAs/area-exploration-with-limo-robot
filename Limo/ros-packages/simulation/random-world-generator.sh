#!/bin/bash


cp -r ./simulation/launch $(rospack find limo_gazebo_sim)
cp -r ./simulation/worlds $(rospack find limo_gazebo_sim)
cp -r ./simulation/param $(rospack find limo_gazebo_sim)

roslaunch limo_gazebo_sim random_world.launch random_world_number:=$((1 + $RANDOM % 3)) &



