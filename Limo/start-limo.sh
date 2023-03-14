#! /bin/bash 

source /opt/ros/noetic/setup.bash

IS_SIMULATION=$1
echo $IS_SIMULATION
LIMO_ID=$2
echo $LIMO_ID

MASTER_IP=$(hostname -I | head -n1 | awk '{print $1;}')
ROS_MASTER_URI="http://${MASTER_IP}:11311"
echo $ROS_MASTER_URI

docker build  -t ros-packages-server ./ros-packages 
docker run --rm --network host -e ROS_MASTER_URI=$ROS_MASTER_URI  -d ros-packages-server

sleep 10

LIMO_IP=$(docker logs ros-packages-server | head -n1 | awk '{print $1;}')
echo $LIMO_IP

docker build  -t ros-server ./ros-server
exec docker run -p 9332:9332 -e LIMO_IP=$LIMO_IP -e IS_SIMULATION=$IS_SIMULATION -e LIMO_ID=$LIMO_ID --rm ros-server
