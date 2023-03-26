import rospy
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID
import os


import roslaunch

class ExplorationControl:
    def __init__(self):

        self.isSimulation = os.environ.get("IS_SIMULATION")

        if self.isSimulation : 
            limoId = os.environ.get("LIMO_ID", "1")
            rospy.init_node('exploration_control' + limoId)

            self.subscriberState = rospy.Subscriber(f"limo{limoId}/exploration_state", Bool, self.setExplorationState)
            self.noMove = rospy.Publisher(f"limo{limoId}/move_base/cancel", GoalID, queue_size=10)

            self.node = roslaunch.core.Node("limo_gazebo_sim", "main_exploration.launch")
        else :
            rospy.init_node('exploration_control')
            self.subscriberState = rospy.Subscriber('/exploration_state', Bool, self.setExplorationState)
            self.noMove = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

            self.node = roslaunch.core.Node("limo_bringup", "one_exploration.launch")

        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()

        self.isExploring = False

    def setExplorationState(self, msg: Bool):
        if self.isExploring == msg.data :
            return

        self.isExploring = msg.data
    
        if self.isExploring :
            self.process = self.launch.launch(self.node)
        else :
            self.stopRobot()

    def stopRobot(self):
        if self.process.is_alive() :
            self.process.stop()

            msg = GoalID()
            self.noMove.publish(msg)


if __name__ == '__main__':
    ec = ExplorationControl()
    rospy.spin()