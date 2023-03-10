import subprocess
import time
import os
import rosnode


def handle_rosbridge():
        
        running_nodes = rosnode.get_node_names()
        if "/rosbridge_websocket" not in running_nodes:
            ROS_rosbridge = subprocess.Popen(['roslaunch', 'rosbridge_server', 'rosbridge_websocket.launch'], preexec_fn=os.setpgrp, stdout=subprocess.DEVNULL)
            print(">> Started ROSBRIDGE.")
        else:
            print(">> ROSBRIDGE already running.")
        
        # webbrowser.open('https://webviz.io/app/')