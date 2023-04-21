#!/bin/bash

source /opt/ros/noetic/setup.bash

echo $(hostname -I)
sleep 5

source /agx_ws/devel/setup.bash --extend

function launch_sequence {
  if [ ! "$IS_SIMULATION" ];  then 
    cp -r ./packages/launchs $(rospack find limo_bringup)
    cp -r ./packages/params $(rospack find limo_bringup)

    roslaunch rosbridge_server rosbridge_websocket.launch &
    sleep 5
    # Launch gmapping
    roslaunch --wait  limo_bringup one_gmapping.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

    # Wait for gmapping to start up
    sleep 5

    # Launch map_merge
    roslaunch --wait  limo_bringup map_merge.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

    # Wait for map_merge to start up
    sleep 5

    # Launch navigation stack
    roslaunch  --wait  limo_bringup one_navigation.launch  2> >(grep -v TF_REPEATED_DATA buffer_core) &

    # Wait for navigation stack to start up
    sleep 5

    # Subscribe to /exploration_state topic to control exploration state
    echo "Launching explore_control..."
    rosrun explore_control control_explore.py  &
    rosrun explore_control return_to_base.py &
  else 
    echo "Launching explore_control..."
    rosrun explore_control control_explore.py &
  fi
}

launch_sequence

exec rosrun update-pkg restart-package-container.py