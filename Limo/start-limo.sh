#! /bin/bash

# Check if input parameter is provided
if [ -z "$1" ]; then
  echo "Error: Missing input parameter (IS_SIMULATION)."
  exit 1
fi

IS_SIMULATION=$1
echo "IS_SIMULATION: $IS_SIMULATION"

if [ "$IS_SIMULATION" -eq 1 ]; then
  if [ -z "$2" ]; then
    echo "Error: Missing input parameter (PATH_TO_SETUP_BASH) when IS_SIMULATION is set to 1. You must provide path for limo_gazebo_sim and limo_bringup setup.bash"
    exit 1
  fi
  PATH_TO_SETUP_BASH=$2
fi

MASTER_IP=$(hostname -I | head -n1 | awk '{print $1;}')
ROS_MASTER_URI="http://${MASTER_IP}:11311"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"

echo "Building Docker Images..."
docker build -t ros-packages-server ./ros-packages || { echo "Error: Failed to build ros-packages-server"; exit 1; } &
docker build -t ros-server ./ros-server || { echo "Error: Failed to build ros-server"; exit 1; } &

wait

echo "Sourcing ${PATH_TO_SETUP_BASH}..."
source ${PATH_TO_SETUP_BASH} --extend || { echo "Error: Failed to source setup.bash"; exit 1; }

if [ "$IS_SIMULATION" == "true" ] || [ "$IS_SIMULATION" == "1" ]; then

  echo "Copying files..."
  cp -r ./ros-packages/simulation/launch $(rospack find limo_gazebo_sim) || { echo "Error: Failed to copy launch files"; exit 1; } &
  cp -r ./ros-packages/simulation/worlds $(rospack find limo_gazebo_sim) || { echo "Error: Failed to copy worlds files"; exit 1; } &
  cp -r ./ros-packages/simulation/params $(rospack find limo_gazebo_sim) || { echo "Error: Failed to copy params files"; exit 1; } &
  cp -r ./ros-packages/packages/launchs $(rospack find limo_bringup) || { echo "Error: Failed to copy launchs files"; exit 1; } &
  cp -r ./ros-packages/packages/params $(rospack find limo_bringup) || { echo "Error: Failed to copy params files"; exit 1; } &

  wait

  echo "Building simulated explore control..."
  pushd ros-packages/packages
  catkin_make
  source devel/setup.bash --extend
  popd

  echo "Running random-world-generator.sh script..."
  ./ros-packages/simulation/random-world-generator.sh > random-world-generator.log 2>&1 || { echo "Error: Failed to run random-world-generator.sh"; exit 1; } &

  wait
  echo "Launching simulated explore control..."
  rosrun explore_control simulation_explore.py &
  sleep 5

  docker run --name ros-packages-server-1  --network host --restart always -e ROS_MASTER_URI=$ROS_MASTER_URI -e IS_SIMULATION=1 -e LIMO_ID='1'  -d ros-packages-server &
  docker run --name ros-packages-server-2  --network host --restart always -e ROS_MASTER_URI=$ROS_MASTER_URI -e IS_SIMULATION=1 -e LIMO_ID='2'  -d ros-packages-server &

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