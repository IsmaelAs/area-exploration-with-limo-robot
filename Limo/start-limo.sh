#! /bin/bash

# Check if input parameter is provided
if [ -z "$1" ]; then
  echo "Error: Missing input parameter (IS_SIMULATION)."
  exit 1
fi

IS_SIMULATION=$1
echo "IS_SIMULATION: $IS_SIMULATION"

MASTER_IP=$(hostname -I | head -n1 | awk '{print $1;}')
ROS_MASTER_URI="http://${MASTER_IP}:11311"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"

echo "Building Docker Images..."
docker build -t ros-packages-server ./ros-packages || { echo "Error: Failed to build ros-packages-server"; exit 1; } &
docker build -t ros-server ./ros-server || { echo "Error: Failed to build ros-server"; exit 1; } &

wait

echo "Sourcing ~/catkin_ws/devel/setup.bash..."
source ~/catkin_ws/devel/setup.bash  || { echo "Error: Failed to source setup.bash"; exit 1; }

if [ "$IS_SIMULATION" == "true" ] || [ "$IS_SIMULATION" == "1" ]; then

  echo "Copying files..."
  cp -r ./ros-packages/simulation/launch $(rospack find limo_gazebo_sim) || { echo "Error: Failed to copy launch files"; exit 1; } &
  cp -r ./ros-packages/simulation/worlds $(rospack find limo_gazebo_sim) || { echo "Error: Failed to copy worlds files"; exit 1; } &
  cp -r ./ros-packages/simulation/params $(rospack find limo_gazebo_sim) || { echo "Error: Failed to copy params files"; exit 1; } &
  cp -r ./ros-packages/packages/launchs $(rospack find limo_bringup) || { echo "Error: Failed to copy launchs files"; exit 1; } &
  cp -r ./ros-packages/packages/params $(rospack find limo_bringup) || { echo "Error: Failed to copy params files"; exit 1; } &

  wait

  echo "Running random-world-generator.sh script..."
  ./ros-packages/simulation/random-world-generator.sh > random-world-generator.log 2>&1 || { echo "Error: Failed to run random-world-generator.sh"; exit 1; } &

  wait

  echo "Starting Docker containers (Simulation mode)..."
  docker run --name ros-packages-server-1 --network host --rm -e ROS_MASTER_URI=$ROS_MASTER_URI -e IS_SIMULATION=1 -e LIMO_ID='1' -d ros-packages-server > ros-packages-server-1.log 2>&1 &
  docker run --name ros-packages-server-2 --network host --rm -e ROS_MASTER_URI=$ROS_MASTER_URI -e IS_SIMULATION=1 -e LIMO_ID='2' -d ros-packages-server > ros-packages-server-2.log 2>&1 &

  sleep 10

  LIMO_IP_SIMU_1=$(grep -m 1 -oP '(?<=LIMO_IP_SIMU_1: )[^\s]+' ros-packages-server-1.log)
  echo "LIMO_IP_SIMU_1: $LIMO_IP_SIMU_1"
  LIMO_IP_SIMU_2=$(grep -m 1 -oP '(?<=LIMO_IP_SIMU_2: )[^\s]+' ros-packages-server-2.log)
  echo "LIMO_IP_SIMU_2: $LIMO_IP_SIMU_2"

  docker run --name ros-server-1 -p 9332:9332 --rm -e LIMO_IP=$LIMO_IP_SIMU_1 -e IS_SIMULATION=1 -e LIMO_ID='1' -d ros-server > ros-server-1.log 2>&1 &
  docker run --name ros-server-2 -p 9333:9333 --rm -e LIMO_IP=$LIMO_IP_SIMU_2 -e IS_SIMULATION=1 -e LIMO_ID='2' ros-server > ros-server-2.log 2>&1 &

else

  echo "Starting Docker containers (Non-simulation mode)..."
  docker run --name ros-packages-server --network host --rm -e ROS_MASTER_URI=$ROS_MASTER_URI -d ros-packages-server > ros-packages-server.log 2>&1 &

  sleep 10

  LIMO_IP=$(grep -m 1 -oP '(?<=LIMO_IP: )[^\s]+' ros-packages-server.log)
  echo "LIMO_IP: $LIMO_IP"

  docker run --name ros-server -p 9332:9332 --rm -e LIMO_IP=$LIMO_IP ros-server > ros-server.log 2>&1 &

fi

wait