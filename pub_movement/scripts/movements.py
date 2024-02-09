#!/usr/bin/env python3
import rospy
import sys
from opencat import roscat
from opencat.roscat import Command, Task
from std_msgs.msg import String

def motion_callback(msg):
    # This function will be called whenever a new message is received on the topic
    # '/motion'

    if msg.data == 'W' or msg.data == 'up' or msg.data == 'UP':
        sc.SendTask(roscat.Task(roscat.Command.WALK, [], 2))
    elif msg.data == 'S' or msg.data == 'down':
        sc.SendTask(roscat.Task(roscat.Command.BACK, [], 2))
    elif msg.data == 'A' or msg.data == 'left':
        sc.SendTask(roscat.Task(roscat.Command.WALK_LEFT, [], 2))
    elif msg.data == 'D' or msg.data == 'right':
        sc.SendTask(roscat.Task(roscat.Command.WALK_RIGHT, [], 2))
    elif msg.data == 'PU':
        sc.SendTask(roscat.Task(roscat.Command.PUSH_UP, [], 2))
    elif msg.data == 'C':
        sc.SendTask(roscat.Task(roscat.Command.CRAWL, [], 2))
    elif msg.data == 'CL':
        sc.SendTask(roscat.Task(roscat.Command.CRAWL_LEFT, [], 2))
    elif msg.data == 'CR':
        sc.SendTask(roscat.Task(roscat.Command.CRAWL_RIGHT, [], 2))
    elif msg.data == 'G':
        sc.SendTask(roscat.Task(roscat.Command.GREETING, [], 2))
    elif msg.data == 'P':
        sc.SendTask(roscat.Task(roscat.Command.PEE, [], 2))
    elif msg.data == 'CALIB':
        sc.SendTask(roscat.Task(roscat.Command.CALIB_POSE, [], 2))
    elif msg.data == 'BF':
        sc.SendTask(roscat.Task(roscat.Command.BACK_FLIP, [], 2))
    else:
        sc.SendTask(roscat.Task(roscat.Command.BALANCE, [], 2))
        

    rospy.loginfo("Received motion: %s", msg.data)


def main():
    rospy.init_node('motion_subscriber', anonymous=True)

    # Define the topic you want to subscribe to and the message type
    topic_name = '/motion'
    message_type = String

    # Create a subscriber that listens to the topic and calls the path_callback function
    rospy.Subscriber(topic_name, message_type, motion_callback)

    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        rospy.logerr("Need robot name")
        sys.exit(1)

    robot_name = str(sys.argv[1])
    sc = roscat.ServiceClient(robot_name)
    main()