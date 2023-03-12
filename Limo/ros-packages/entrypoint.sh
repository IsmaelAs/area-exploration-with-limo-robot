#! /bin/bash

source /opt/ros/noetic/setup.bash 
echo $(hostname -I)
exec roslaunch rosbridge_server rosbridge_websocket.launch 
