#! /bin/bash 

IS_SIMULATION=$1
echo "is simulation  = $IS_SIMULATION"
SIMULATION_FOLDER=$2
echo  "'$SIMULATION_FOLDER'"

MASTER_IP=$(hostname -I | head -n1 | awk '{print $1;}')
ROS_MASTER_URI="http://${MASTER_IP}:11311"
echo $ROS_MASTER_URI


if ([ ! -z "$SIMULATION_FOLDER" ] && [ "$IS_SIMUALTION" == "true" ]) || ([ ! -z "$SIMULATION_FOLDER" ] && [ "$IS_SIMULATION" == "1" ]); then 

  docker build  -t ros-packages-server ./ros-packages 
  docker build  -t ros-server ./ros-server

  source $SIMULATION_FOLDER
  
  cp -r ./ros-packages/simulation/launch $(rospack find limo_gazebo_sim)
  cp -r ./ros-packages/simulation/worlds $(rospack find limo_gazebo_sim)
  cp -r ./ros-packages/simulation/params $(rospack find limo_gazebo_sim)
  cp -r ./ros-packages/packages/launchs $(rospack find limo_bringup)
  cp -r ./ros-packages/packages/params $(rospack find limo_bringup)

  ./ros-packages/simulation/random-world-generator.sh

  docker run --name ros-packages-server-1  --network host --restart always -e ROS_MASTER_URI=$ROS_MASTER_URI -e IS_SIMULATION=1 -e LIMO_ID='1'  -d ros-packages-server
  docker run --name ros-packages-server-2  --network host --restart always -e ROS_MASTER_URI=$ROS_MASTER_URI -e IS_SIMULATION=1 -e LIMO_ID='2'  -d ros-packages-server

  sleep 10

  LIMO_IP_SIMU_1=$(docker logs ros-packages-server-1 | head -n1 | awk '{print $1;}')
  echo $LIMO_IP_SIMU_1
  LIMO_IP_SIMU_2=$(docker logs ros-packages-server-2 | head -n1 | awk '{print $1;}')
  echo $LIMO_IP_SIMU_2

  docker run --name ros-server-1 -p 9332:9332 --restart always -e LIMO_IP=$LIMO_IP_SIMU_1 -e IS_SIMULATION=1 -e LIMO_ID='1'  -d ros-server
  docker run --name ros-server-2 -p 9333:9333 --restart always -e LIMO_IP=$LIMO_IP_SIMU_2 -e IS_SIMULATION=1 -e LIMO_ID='2'   ros-server


elif [ "$IS_SIMULATION" == "" ]; then

  docker build  -t ros-packages-server ./ros-packages 
  docker build  -t ros-server ./ros-server

  docker run --name ros-packages-server  --network host --restart always -e ROS_MASTER_URI=$ROS_MASTER_URI -d ros-packages-server
  sleep 10

  LIMO_IP=$(docker logs ros-packages-server | head -n1 | awk '{print $1;}')
  echo $LIMO_IP

  docker run --name ros-server -p 9332:9332 --restart always -e LIMO_IP=$LIMO_IP   ros-server

else 

  echo "Le bon format pour lancer la simulation est : sudo ./start-limo.sh 1 < Chemin du dossier de dependance gazebo >"

fi 
