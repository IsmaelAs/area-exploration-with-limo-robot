import rospy
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal
import os
import random

class ExplorationControl:
    def __init__(self):

        isSimulation = os.environ.get("IS_SIMULATION")

        if isSimulation : 
            limoId = os.environ.get("LIMO_ID", "1")
            rospy.init_node('exploration_control' + limoId)
            self.subscriberState = rospy.Subscriber(f"limo{limoId}/exploration_state", Bool, self.setExplorationState)
            self.noMove = rospy.Publisher(f"limo{limoId}/move_base/cancel", GoalID, queue_size=10)
            self.sendGoal = rospy.Publisher(f"limo{limoId}/move_base/goal", MoveBaseActionGoal, queue_size=10)
        else :
            rospy.init_node('exploration_control')
            self.subscriberState = rospy.Subscriber('/exploration_state', Bool, self.setExplorationState)
            self.noMove = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
            self.sendGoal = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)



        self.isExploring = False
        self.rate = rospy.Rate(1)
        self.stopRobot()

    def setExplorationState(self, msg: Bool):
        if msg.data != self.isExploring:

            self.isExploring = msg.data
            
            if(not self.isExploring):
                self.stopRobot()

    def stopRobot(self):

        while not self.isExploring :
            msg = GoalID()
            self.noMove.publish(msg)
            self.rate.sleep()
        
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.pose.position.x = random.randint(0, 3)
        goal.goal.target_pose.pose.position.y = random.randint(0, 3)
        goal.goal.target_pose.pose.orientation.w = 0.66

        self.sendGoal.publish(goal)

    

if __name__ == '__main__':
    ec = ExplorationControl()
    rospy.spin()