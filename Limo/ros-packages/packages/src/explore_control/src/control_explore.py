import rospy
from std_msgs.msg import Bool
import os
import subprocess

class ExplorationControl:
    def __init__(self):
        rospy.init_node('exploration_control')
        self.subscriberState = rospy.Subscriber(
                '/exploration_state', Bool, self.setExplorationState)
        self.isExploring = False
        self.explore_lite_process = None

    def setExplorationState(self, msg: Bool):
        if msg.data != self.isExploring:
            self.isExploring = msg.data

            if self.isExploring:
                self.launch_explore_lite()
            else:
                self.stop_explore_lite()

    def launch_explore_lite(self):
        if self.explore_lite_process is None:
            self.explore_lite_process = subprocess.Popen(
                ["roslaunch", "limo_gazebo_sim", "main_exploration.launch"],
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