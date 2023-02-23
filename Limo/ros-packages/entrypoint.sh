#! /bin/bash

source /opt/ros/noetic/setup.bash 
hostname -I

ROS_IP=$(hostname -I | head -n1 | awk '{print $1;}')
# echo $ROS_IP
# export ROS_IP=$ROS_IP
export ROS_HOSTNAME=$ROS_IP
# export ROS_HOSTNAME=ubuntu.local


roslaunch rosbridge_server rosbridge_websocket.launch 
