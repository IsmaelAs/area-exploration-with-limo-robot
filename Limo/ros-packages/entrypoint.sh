#!/bin/bash

source /opt/ros/noetic/setup.bash
echo $(hostname -I)

# Launch gmapping
xterm -e "roslaunch limo_navigation gmapping.launch" &

# Wait for gmapping to start up
sleep 5

# Launch AMCL
xterm -e "roslaunch limo_navigation amcl.launch" &

# Wait for AMCL to start up
sleep 5

# Launch move_base
xterm -e "roslaunch limo_navigation move_base.launch" &

# Wait for move_base to start up
sleep 5

# Launch explore_lite
xterm -e "roslaunch limo_navigation explore_costmap.launch" &
