#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def keyboard_input_publisher():
    # Initialize the ROS node
    rospy.init_node('keyboard_input_node', anonymous=True)

    # Create a publisher for the /motion topic
    motion_publisher = rospy.Publisher('/motion', String, queue_size=10)

    # Loop to read keyboard inputs and publish them
    while not rospy.is_shutdown():
        # Get user input from the terminal
        user_input = input("Enter motion command: ").upper()

        # Create a String message and publish it to the /motion topic
        motion_msg = String()
        motion_msg.data = user_input
        motion_publisher.publish(motion_msg)

if __name__ == '__main__':
    try:
        keyboard_input_publisher()
    except rospy.ROSInterruptException:
        pass
