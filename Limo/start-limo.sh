#! /bin/bash 

source /opt/ros/noetic/setup.bash

IS_SIMULATION=$1
LIMO_ID=$2

echo $IS_SIMULATION
echo $LIMO_ID

MASTER_IP=$(hostname -I | head -n1 | awk '{print $1;}')
ROS_MASTER_URI="http://${MASTER_IP}:11311"
echo $ROS_MASTER_URI


docker build  -t ros-packages-server ./ros-packages 
docker build  -t ros-server ./ros-server



docker run --name ros-packages-server  --network host --restart always -e LIMO_ID=$LIMO_ID -e ROS_MASTER_URI=$ROS_MASTER_URI -e IS_SIMULATION=$IS_SIMULATION -d ros-packages-server
sleep 10

LIMO_IP=$(docker logs ros-packages-server | head -n1 | awk '{print $1;}')
echo $LIMO_IP

exec docker run --name ros-server -p 9332:9332 --restart always -e LIMO_ID=$LIMO_ID -e LIMO_IP=$LIMO_IP -e IS_SIMULATION=$IS_SIMULATION  ros-server
