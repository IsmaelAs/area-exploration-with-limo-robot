#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import os
import subprocess
from explore_control.msg import BoolString

class ExplorationControl:
    def __init__(self):
        rospy.init_node('simulation_explore')
        self.subscriberState = rospy.Subscriber(f"/exploration_state_sim", BoolString, self.setExplorationState)
        self.explore_lite_processes = {}


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
            self.explore_lite_processes[f'{msg.info}'].terminate()
            del self.explore_lite_processes[f'{msg.info}']

if __name__ == '__main__':
    ec = ExplorationControl()
    rospy.spin()
