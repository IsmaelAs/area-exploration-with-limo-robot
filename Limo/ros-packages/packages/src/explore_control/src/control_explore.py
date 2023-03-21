import rospy
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID
import os


class ExplorationControl:
    def __init__(self):
        rospy.init_node('exploration_control')


        self.subscriberState = rospy.Subscriber('/exploration_state', Bool, self.setExplorationState)
        self.noMove = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)


        isSimulation = os.environ.get("IS_SIMULATION")

        if isSimulation : 
            limoId = os.environ.get("LIMO_ID", "1")
            self.subscriberState = rospy.Subscriber(f'limo{limoId}/exploration_state', Bool, self.setExplorationState)
            self.noMove = rospy.Publisher('limo{limoId}/move_base/cancel', GoalID, queue_size=10)
        

        self.isExploring = False
        self.rate = rospy.Rate(1)

        rospy.loginfo('Tout est crée')
        self.stopRobot()

    def setExplorationState(self, msg: Bool):
        rospy.loginfo("recu message pour set ; dans set stat")
        if msg.data != self.isExploring:

            rospy.loginfo(f"set l'exploration a : {msg.data}")
            self.isExploring = msg.data
            
            if(not self.isExploring):
                rospy.loginfo("commencement de l'arrêt des goals")
                self.stopRobot()

    def stopRobot(self):
        rospy.loginfo("dans stop robot")

        while not self.isExploring :
            msg = GoalID()
            self.noMove.publish(msg)
            rospy.loginfo("envoie du message d'arret des goal")
            self.rate.sleep()

    

if __name__ == '__main__':
    ec = ExplorationControl()
    rospy.spin()