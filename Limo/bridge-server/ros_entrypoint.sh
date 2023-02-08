#! /bin/bash

export ROS_MASTER_URI=http://$LIMO_IP:11311

echo $ROS_MASTER_URI

source /opt/ros/noetic/setup.bash
bash

