#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def image_callback(msg):
    try:
        # Convert the ROS CompressedImage message to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        print(e)

def main():
    # Initialize the ROS node
    rospy.init_node('image_viewer_node', anonymous=True)

    # Create a CvBridge instance
    global bridge
    bridge = CvBridge()

    # Subscribe to the compressed image topic
    image_topic = "/midog3/camera/image_rect_color/compressed"  # Adjust the topic name
    rospy.Subscriber(image_topic, CompressedImage, image_callback, queue_size=10)

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    main()
