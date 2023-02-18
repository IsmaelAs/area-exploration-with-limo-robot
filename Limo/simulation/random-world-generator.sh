#!/bin/bash


roslaunch ./launch/random_world.launch random_world_number:=$((1 + $RANDOM % 3))
