#!/usr/bin/env python

import rospy
import subprocess
import signal
import roslaunch
from std_srvs.srv import Empty
from deliverybot_navigation.srv import MapFilePath, MapFilePathResponse

map_server = None

def set_map(req):
    global map_server
    if map_server is not None:
        map_server.send_signal(signal.SIGINT)
    map_server = subprocess.Popen(["rosrun", "map_server", "map_server", req.map_file_path])
    return True
  
    

if __name__ == "__main__":
    rospy.init_node("navigation_map_server")
    rospy.Service('load_map', MapFilePath, set_map)
    rospy.spin()