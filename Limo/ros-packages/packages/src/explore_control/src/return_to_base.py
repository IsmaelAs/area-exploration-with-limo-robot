#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry  # Add this import
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
import os
import sys

class ReturnToBase:
    def __init__(self):
        self.base_pose = None
        self.isSimulation = os.environ.get("IS_SIMULATION")
        if self.isSimulation : 
            self.limoId = os.environ.get("LIMO_ID", "1")
            rospy.init_node('return_to_base_node' + self.limoId)

            self.pose_subscriber = rospy.Subscriber(f'/limo{self.limoId}/odom', Odometry, self.pose_callback)  # Update the message type
            self.return_subscriber = rospy.Subscriber(f'/limo{self.limoId}/return_to_base', Bool, self.return_callback)

            self.move_base_client = actionlib.SimpleActionClient(f'/limo{self.limoId}/move_base', MoveBaseAction)
            self.move_base_client.wait_for_server()
        else:
            rospy.init_node('return_to_base_node')

            self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.pose_callback)  # Update the message type
            self.return_subscriber = rospy.Subscriber('/return_to_base', Bool, self.return_callback)

            self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            self.move_base_client.wait_for_server()

    def pose_callback(self, msg):
        if self.base_pose is None:
            self.base_pose = msg.pose.pose  # Update this line to get the pose from the Odometry message

    def return_callback(self, msg):
        if msg.data and self.base_pose is not None:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.base_pose

            self.move_base_client.send_goal(goal)
            result = self.move_base_client.wait_for_result()

            if result and self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Robot returned to base successfully")
            else:
                rospy.logwarn("Failed to return to base")
            

if __name__ == '__main__':
    return_to_base = ReturnToBase()
    rospy.spin()