#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import os
import subprocess
from explore_control.msg import BoolString
from actionlib_msgs.msg import GoalID

class ExplorationControl:
    def __init__(self):
        self.isSimulation = os.environ.get("IS_SIMULATION")
        if self.isSimulation : 
            self.limoId = os.environ.get("LIMO_ID", "1")
            rospy.init_node('exploration_control' + self.limoId)
            self.subscriberState = rospy.Subscriber(f"/limo{self.limoId}/exploration_state", Bool, self.setExplorationState)
            self.publishState = rospy.Publisher('/exploration_state_sim', BoolString, queue_size=10)
        else:
            rospy.init_node('exploration_control')
            self.subscriberState = rospy.Subscriber(
                    '/exploration_state', Bool, self.setExplorationState)
            self.move_base_cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.return_to_base_process = None
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
                new_msg = BoolString()
                new_msg.data = True
                new_msg.info = f'{self.limoId}'
                self.publishState.publish(new_msg)
                self.return_to_base_process = subprocess.Popen(["rosrun", "explore_control", "return_to_base.py"], stderr=subprocess.PIPE, preexec_fn=os.setpgrp)
                # Wait for the process to complete and get the output and error messages
                stdout, stderr = self.return_to_base_process.communicate()

                # Check the return code of the command
                if self.return_to_base_process.returncode == 0:
                    print("Return to base node deployed")
                else:
                    print("Error deploying return to base node:")
                    print(stderr.decode("utf-8"))
            else:
                rospy.loginfo("Launching explore_control for physical limo")
                self.explore_lite_process = subprocess.Popen(
                    ["roslaunch", 'limo_bringup', "one_exploration.launch"],
                    stderr=subprocess.PIPE, preexec_fn=os.setpgrp)
                self.warning_filter_process = subprocess.Popen(
                    ["grep", "-v", "TF_REPEATED_DATA", "buffer_core"],
                    stdin=self.explore_lite_process.stderr)
                
                self.return_to_base_process = subprocess.Popen(["rosrun", "explore_control", "return_to_base.py"], stderr=subprocess.PIPE, preexec_fn=os.setpgrp)

    def stop_explore_lite(self):
        if self.isSimulation:
            new_msg = BoolString()
            new_msg.data = False
            new_msg.info = f'{self.limoId}'
            self.publishState.publish(new_msg)
            self.return_to_base_process.terminate()
        else:
            if self.explore_lite_process is not None:
                map_save_process = subprocess.Popen(
                ["python3", "/agx_ws/src/explore_control/src/save_map.py"],
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
                self.explore_lite_process.terminate()
                self.explore_lite_process = None
                self.return_to_base_process.terminate()
                self.move_base_cancel_pub.publish(GoalID())
                map_save_process.terminate()

if __name__ == '__main__':
    ec = ExplorationControl()
    rospy.spin()
