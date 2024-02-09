#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        # Convert the CompressedImage to a CV2 image
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Now you can use cv_image as a regular OpenCV image
        cv2.imshow("Image", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        print(e)

def main():
    rospy.init_node('image_listener', anonymous=True)
    image_topic = "/midog3/camera/image_rect_color/compressed"  # Update with your actual topic
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
