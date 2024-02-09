#!/usr/bin/env python3
import rospy
import sys
from opencat import roscat

def initialize_node():
    # Initialize the ROS node
    rospy.init_node('Controller_node')

def send_tasks(robot_name):
    # Create a ServiceClient
    sc = roscat.ServiceClient(robot_name)

    # Send tasks
    # sc.SendTask(roscat.Task(roscat.Command.CALIB_POSE, [], 2))
    # sc.SendTask(roscat.Task(roscat.Command.PEE, [], 2))
    sc.SendTask(roscat.Task(roscat.Command.GREETING, [], 2))

if __name__ == "__main__":
    if len(sys.argv) != 2:
        rospy.logerr("Need robot name")
        sys.exit(1)

    robot_name = str(sys.argv[1])
    
    initialize_node()
    send_tasks(robot_name)
