#!/usr/bin/env python3

from std_msgs.msg import Bool
import rospy 
import os


class RestartContainer : 
    
    def __init__(self) -> None:
        isSimulation = os.environ.get("IS_SIMULATION")

        if isSimulation:
            limoId = os.environ.get("LIMO_ID", "1")
            rospy.init_node('package_server_restart' + limoId)
            self.subsciber = rospy.Subscriber(f"limo{limoId}/restartPackagesContainer", Bool, self.stopContainer)
        else :
            rospy.init_node('package_server_restart')
            self.subsciber = rospy.Subscriber(f"restartPackagesContainer", Bool, self.stopContainer)


    def stopContainer(self, data_): 
        rospy.loginfo("Arret de la node et do conteneur !")
        os._exit(0)


if __name__ == "__main__" :
    rc = RestartContainer()
    rospy.spin()