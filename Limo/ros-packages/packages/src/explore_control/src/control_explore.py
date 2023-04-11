#! /bin/python3

import rospy
from std_msgs.msg import Bool
import os
import subprocess

class ExplorationControl:
    def __init__(self):
        self.isSimulation = os.environ.get("IS_SIMULATION")

        if self.isSimulation : 
            self.limoId = os.environ.get("LIMO_ID", "1")
            rospy.init_node('exploration_control' + self.limoId)
            self.subscriberState = rospy.Subscriber(f"limo{self.limoId}/exploration_state", Bool, self.setExplorationState)
        else:
            rospy.init_node('exploration_control')
            self.subscriberState = rospy.Subscriber(
                    '/exploration_state', Bool, self.setExplorationState)
        self.explore_lite_process = None
        rospy.loginfo("fini le init")


    def setExplorationState(self, msg: Bool):
            rospy.loginfo("callback")
            if msg.data == True:
                self.launch_explore_lite()
            else:
                self.stop_explore_lite()

    def launch_explore_lite(self):
        if self.explore_lite_process is None:
            if self.isSimulation:
                rospy.loginfo("Launching explore_control for simulated limo")
                self.explore_lite_process = subprocess.Popen(
                    ["roslaunch", "--wait", "limo_gazebo_sim", "one_exploration.launch", f'ns:=/limo{self.limoId}', f'id:=limo{self.limoId}'],
                    stderr=subprocess.PIPE, preexec_fn=os.setpgrp)
            else:
                rospy.loginfo("Launching explore_control for physical limo")
                self.explore_lite_process = subprocess.Popen(
                    ["roslaunch", "limo_bringup", "one_exploration.launch"],
                    stderr=subprocess.PIPE, preexec_fn=os.setpgrp)
            self.warning_filter_process = subprocess.Popen(
                ["grep", "-v", "TF_REPEATED_DATA", "buffer_core"],
                stdin=self.explore_lite_process.stderr)

    def stop_explore_lite(self):
        if self.explore_lite_process is not None:
            self.explore_lite_process.terminate()
            self.explore_lite_process = None

if __name__ == '__main__':
    ec = ExplorationControl()
    rospy.spin()