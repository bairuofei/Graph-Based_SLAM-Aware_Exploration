#!/usr/bin/python3
'''
Author: ruofei_ntu 991609404@qq.com
Date: 2023-06-21 19:29:05
LastEditors: ruofei_ntu 991609404@qq.com
LastEditTime: 2023-06-22 10:36:30
FilePath: /catkin_cpp_ws/src/cpp_solver/scripts/exploration_button_client.py
Description: 

Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
'''
import sys
import rospy
from cpp_solver.srv import ExplorationButton

def usage():
    return "%s [value]"%sys.argv[0]

def exploration_button_client(value: bool):
    rospy.wait_for_service('explore_button')
    try:
        # Return the service function
        service_func = rospy.ServiceProxy('explore_button', ExplorationButton)
        ret = service_func(value)
        if ret:
            print(f"Send explore button: {value} ")
        else:
            print("Service call failed: %s"%e)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    if len(sys.argv) == 2:
        if sys.argv[1] == "true" or sys.argv[1] == "True":
            value = True
        else:
            value = False
    else:
        print(usage())
        sys.exit(1)  
    exploration_button_client(value)