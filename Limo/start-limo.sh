#! /bin/bash 

source /opt/ros/noetic/setup.bash

# source ~/simu/devel/setup.bash

# roslaunch limo_gazebo_sim limo_ackerman.launch &
# ROS=$(ipconfig.exe | grep 'Wi-Fi' -a7 | cut -d":" -f 2  | tail -n1 | sed -e 's/\s*//g')
# echo $ROS
MASTER_IP=$(hostname -I | head -n1 | awk '{print $1;}')
ROS_MASTER_URI="http://${MASTER_IP}:11311"
echo $ROS_MASTER_URI
docker build  -t ros-packages-server ./ros-packages 
docker run --name ros-packages-server --rm --network host -e ROS_MASTER_URI=$ROS_MASTER_URI  -d ros-packages-server
# ROS2="http://${ROS}:11311"
# echo $ROS2
# LIMO_IP=$(ipconfig.exe | grep -a 'Wi-Fi' -A 4 | grep -a 'IPv4' | cut -d":" -f 2  | tail -n1 | sed -e 's/\s*//g')
sleep 10
LIMO_IP=$(docker logs ros-packages-server | head -n1 | awk '{print $1;}')
echo $LIMO_IP
docker build  -t ros-server ./ros-server
docker run --name ros-server -p 9332:9332 -e LIMO_IP=$LIMO_IP --rm ros-server
# echo $ROS_MASTER_URI
# docker-compose -f limo-docker-compose.yml build 
# ROS_MASTER_URI=$LIMO_IP docker-compose -f limo-docker-compose.yml up 
