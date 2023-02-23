#! /bin/bash

source /opt/ros/noetic/setup.bash 
hostname -I


roslaunch rosbridge_server rosbridge_websocket.launch 
