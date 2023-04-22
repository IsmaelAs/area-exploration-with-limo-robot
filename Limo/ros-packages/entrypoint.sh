#!/bin/bash

source /opt/ros/noetic/setup.bash
source /agx_ws/devel/setup.bash --extend
echo $(hostname -I)
sleep 5

source /agx_ws/devel/setup.bash --extend

function launch_sequence {
  if [ ! "$IS_SIMULATION" ];  then 
    echo "Copying launch files and params..."
    cp -r ./packages/launchs $(rospack find limo_bringup)
    cp -r ./packages/params $(rospack find limo_bringup)

    echo "Launching rosbridge_server..."
    roslaunch rosbridge_server rosbridge_websocket.launch &
    sleep 5
    
    echo "Launching gmapping..."
    roslaunch --wait  limo_bringup one_gmapping.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &
    sleep 5

    echo "Launching map_merge..."
    roslaunch --wait  limo_bringup map_merge.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &
    sleep 5

    echo "Launching navigation stack..."
    roslaunch --wait  limo_bringup limo_navigation_ackerman.launch &
    sleep 5

    echo "Launching explore_control..."
    rosrun explore_control control_explore.py  &
    rosrun explore_control return_to_base.py &
  else 
    echo "Launching explore_control in simulation mode..."
    rosrun explore_control control_explore.py &
  fi
}

launch_sequence

echo "Executing restart-package-container.py"
exec rosrun update-pkg restart-package-container.py