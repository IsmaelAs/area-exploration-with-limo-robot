#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import os

class ReturnToBase:
    def __init__(self):
        self.start_pose = None
        self.isSimulation = os.environ.get("IS_SIMULATION")
        if self.isSimulation :
            self.limoId = os.environ.get("LIMO_ID", "1")
            rospy.init_node('return_to_base_node' + self.limoId)

            self.pose_subscriber = rospy.Subscriber(f'/limo{self.limoId}/odom', Odometry, self.pose_callback)
            self.return_subscriber = rospy.Subscriber(f'/limo{self.limoId}/return_to_base', Bool, self.return_callback)
            self.explore_lite_subscriber = rospy.Subscriber(f'/limo{self.limoId}/exploration_state', Bool, self.explore_lite_callback)

            self.goal_publisher = rospy.Publisher(f'/limo{self.limoId}/move_base_simple/goal', PoseStamped, queue_size=10)
            self.stop_explore_lite_publisher = rospy.Publisher(f'/limo{self.limoId}/exploration_state', Bool, queue_size=10)
        else:
            rospy.init_node('return_to_base_node')

            self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.pose_callback)
            self.return_subscriber = rospy.Subscriber('/return_to_base', Bool, self.return_callback)
            self.explore_lite_subscriber = rospy.Subscriber('/exploration_state', Bool, self.explore_lite_callback)

            self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
            self.stop_explore_lite_publisher = rospy.Publisher('/exploration_state', Bool, queue_size=10)

    def pose_callback(self, msg):
        if self.start_pose is None:
            self.start_pose = msg.pose.pose

    def return_callback(self, msg):
        if msg.data and self.start_pose is not None:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose = self.start_pose

            self.stop_explore_lite_publisher.publish(False)  # Stop explore_lite before returning to base
            rospy.sleep(5)
            self.goal_publisher.publish(goal)

    def explore_lite_callback(self, msg):
        if msg.data:
            self.start_pose = None  # Reset the start_pose to record the new position before exploration starts

if __name__ == '__main__':
    return_to_base = ReturnToBase()
    rospy.spin()