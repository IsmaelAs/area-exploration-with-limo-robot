#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import os
import subprocess
from explore_control.msg import BoolString
from datetime import datetime
from actionlib_msgs.msg import GoalID

class ExplorationControl:
    def __init__(self):
        rospy.init_node('simulation_explore')
        self.subscriberState = rospy.Subscriber(f"/exploration_state_sim", BoolString, self.setExplorationState)
        self.explore_lite_processes = {}
        self.return_to_base_process = None


    def setExplorationState(self, msg: BoolString):
            rospy.loginfo("callback")
            if msg.data == True:
                self.launch_explore_lite(msg)
            else:
                self.stop_explore_lite(msg)

    def launch_explore_lite(self, msg: BoolString):
        if msg.info not in self.explore_lite_processes:
            rospy.loginfo("Launching explore_control for simulated limo...")
            self.explore_lite_processes[msg.info] = subprocess.Popen(
                ["roslaunch", 'limo_gazebo_sim', "one_exploration.launch", f'ns:=/limo{msg.info}', f'id:=limo{msg.info}'],
                stderr=subprocess.PIPE, preexec_fn=os.setpgrp)

    def stop_explore_lite(self, msg: BoolString):
        if msg.info in self.explore_lite_processes:
            rospy.loginfo("Stopping explore_control for simulated limo...")
            move_base_cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
            map_save_process = subprocess.Popen(
              ["python3", "./ros-packages/packages/src/explore_control/src/save_map.py", '-s', f"{msg.info}"],
              stderr=subprocess.PIPE, preexec_fn=os.setpgrp  
            )
            # Wait for the process to complete and get the output and error messages
            stdout, stderr = map_save_process.communicate()

            # Check the return code of the command
            if map_save_process.returncode == 0:
                print("Map saved successfully")
            else:
                print("Error saving map:")
                print(stderr.decode("utf-8"))
            self.explore_lite_processes[f'{msg.info}'].terminate()
            del self.explore_lite_processes[f'{msg.info}']
            move_base_cancel_pub.publish(GoalID())

if __name__ == '__main__':
    ec = ExplorationControl()
    rospy.spin()