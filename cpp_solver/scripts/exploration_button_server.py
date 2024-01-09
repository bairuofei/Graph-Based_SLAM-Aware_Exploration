#!/usr/bin/python3

'''
Author: ruofei_ntu 991609404@qq.com
Date: 2023-06-21 10:38:10
LastEditors: ruofei_ntu 991609404@qq.com
LastEditTime: 2023-06-21 19:46:26
FilePath: /catkin_cpp_ws/src/cpp_solver/scripts/exploration_button_server.py
Description: 

Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
'''

import rospy
from cpp_solver.srv import ExplorationButton, ExplorationButtonResponse
from std_msgs.msg import Bool

class ExploreButtonServer:
    def __init__(self, pubStop):
        self.pubStop = pubStop

    def handle_exploration_button(self, req):
        msg = Bool()
        if req.value:
            msg.data = True
        else:
            msg.data = False
        self.pubStop.publish(msg)
        rospy.loginfo(f"Publish exploration button: {msg.data}")
        return ExplorationButtonResponse(True)


if __name__ == "__main__":

    rospy.init_node('exploration_button_server')

    pubStop = rospy.Publisher('stop_exploration', Bool, queue_size=10)
    server_node = ExploreButtonServer(pubStop)

    s = rospy.Service('explore_button', ExplorationButton, server_node.handle_exploration_button)
    print("Start exploration_button_server.")
    rospy.spin()
