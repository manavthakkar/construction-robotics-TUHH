#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

# Initialize MediaPipe Holistic
mp_holistic = mp.solutions.holistic
holistic = mp_holistic.Holistic()

# Initialize MediaPipe Drawing
mp_drawing = mp.solutions.drawing_utils

def image_callback(msg):
    try:
        # Convert the CompressedImage to a CV2 image
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Flip the frame horizontally for a later selfie-view display
        cv_image = cv2.flip(cv_image, 1)

        # Convert the BGR image to RGB
        rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Process the frame with MediaPipe Holistic
        results = holistic.process(rgb_frame)

        # Draw landmarks on the frame
        mp_drawing.draw_landmarks(cv_image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)
        mp_drawing.draw_landmarks(cv_image, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
        mp_drawing.draw_landmarks(cv_image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)

        # Display the resulting frame
        cv2.imshow("MediaPipe Holistic", cv_image)
        cv2.waitKey(1)

    except Exception as e:
        print(e)

def main():
    rospy.init_node('image_listener', anonymous=True)
    image_topic = "/midog3/camera/image_raw/compressed"  # Update with your actual topic
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
