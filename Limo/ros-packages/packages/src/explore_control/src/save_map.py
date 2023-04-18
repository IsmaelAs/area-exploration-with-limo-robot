import os
import subprocess
import sys
from datetime import datetime
now = datetime.now()
formatted_time = now.strftime("%m-%d-%Y-%H-%M-%S")
mission_directory_path = f"missions/limo-mission"
if sys.argv[1] == '-s':   
    mission_directory_path = f"missions/limo{sys.argv[2]}-sim-mission"
create_saving_directory = subprocess.Popen(["mkdir", "-p", mission_directory_path], stderr=subprocess.PIPE, preexec_fn=os.setpgrp)

map_topic = '/map'
if sys.argv[1] == '-s':
    map_topic = f'limo{sys.argv[2]}/map' 

map_save_process = subprocess.Popen(
    ["rosrun", "map_server", "map_saver", "-f", f"{mission_directory_path}/mission", f"map:={map_topic}"], 
    stderr=subprocess.PIPE, preexec_fn=os.setpgrp)

# #!/usr/bin/env python3

# import rospy
# from std_msgs.msg import BoolString
# import os
# import subprocess
# import sys
# from datetime import datetime

# class MapSaver:
#     def __init__(self):
#         rospy.init_node('map_saver')
#         self.sim_saver = rospy.Subscriber(f"/save_map_sim", BoolString, self.save_map_sim)
#         self.limo_saver = rospy.Subscriber(f"/save_map", BoolString, self.save_map)

#     def save_map_sim(self, msg: BoolString):
#         rospy.loginfo("Saving map for simulation")
#         now = datetime.now()
#         formatted_time = now.strftime("%m-%d-%Y-%H-%M-%S")
#         mission_directory_path = f"missions/limo{msg.info}-sim-mission-{formatted_time}"
#         map_topic = f'limo{msg.info}/map' 
#         mission_name = f'mission-{formatted_time}'
#         self.save(mission_directory_path, mission_name, map_topic)

#     def save_map(self, msg: BoolString):
#         rospy.loginfo("Saving map")
#         now = datetime.now()
#         formatted_time = now.strftime("%m-%d-%Y-%H-%M-%S")
#         mission_directory_path = f"missions/limo-sim-mission-{formatted_time}"
#         map_topic = '/map' 
#         mission_name = f'mission-{formatted_time}'
#         self.save(mission_directory_path, mission_name, map_topic)

#     def save(self, mission_directory_path, mission_name, map_topic):
#         create_saving_directory = subprocess.Popen(["mkdir", "-p", mission_directory_path], stderr=subprocess.PIPE, preexec_fn=os.setpgrp)
#         map_save_process = subprocess.Popen(
#             ["rosrun", "map_server", "map_saver", "-f", f"{mission_directory_path}/{mission_name}", f"map:={map_topic}"], 
#            stderr=subprocess.PIPE, preexec_fn=os.setpgrp)

# if __name__ == '__main__':
#     ms = MapSaver()
#     rospy.spin()
