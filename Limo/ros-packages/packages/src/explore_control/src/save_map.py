import os
import subprocess
import sys
from datetime import datetime
now = datetime.now()
formatted_time = now.strftime("%m-%d-%Y-%H-%M-%S")
mission_directory_path = f"missions/limo-mission-{formatted_time}"
if sys.argv[1] == '-s':   
    mission_directory_path = f"missions/limo{sys.argv[2]}-sim-mission-{formatted_time}"
create_saving_directory = subprocess.Popen(["mkdir", "-p", mission_directory_path], stderr=subprocess.PIPE, preexec_fn=os.setpgrp)

map_topic = '/map'
if sys.argv[1] == '-s':
    map_topic = f'limo{sys.argv[2]}/map' 

map_save_process = subprocess.Popen(
    ["rosrun", "map_server", "map_saver", "-f", f"{mission_directory_path}/mission-{formatted_time}", f"map:={map_topic}"], 
    stderr=subprocess.PIPE, preexec_fn=os.setpgrp)