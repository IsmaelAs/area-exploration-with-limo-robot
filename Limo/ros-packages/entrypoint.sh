#! /bin/bash

source /opt/ros/noetic/setup.bash 
echo $(hostname -I)

if [ "$LIMO_ID" != "2" ]; then 
    exec roslaunch rosbridge_server rosbridge_websocket.launch 
fi
