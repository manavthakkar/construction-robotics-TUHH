#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

def image_callback(msg):
    try:
        # Convert the CompressedImage to a CV2 image
        bridge = CvBridge()
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Initialize MediaPipe Pose model
        mp_pose = mp.solutions.pose
        pose = mp_pose.Pose()

        # Convert the frame to RGB
        frame_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Make detection
        results = pose.process(frame_rgb)

        # Check if any pose is detected
        if results.pose_landmarks:
            # Extract landmarks for left and right hip (landmarks 23 and 24)
            left_hip = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP.value]
            right_hip = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP.value]

            # Calculate the midpoint between left and right hip
            midpoint_x = int((left_hip.x + right_hip.x) / 2 * cv_image.shape[1])
            midpoint_y = int((left_hip.y + right_hip.y) / 2 * cv_image.shape[0])

            # Publish the midpoint coordinates to a ROS topic
            midpoint_msg = CompressedImage()
            midpoint_msg.header.stamp = rospy.Time.now()
            midpoint_msg.data = f"{midpoint_x},{midpoint_y}".encode('utf-8')
            midpoint_pub.publish(midpoint_msg)

    except Exception as e:
        print(e)

def main():
    global midpoint_pub
    rospy.init_node('pose_midpoint_publisher', anonymous=True)
    image_topic = "/midog1/camera/image_raw/compressed"  # Update with your actual topic
    rospy.Subscriber(image_topic, CompressedImage, image_callback)
    midpoint_pub = rospy.Publisher("/pose_midpoint", CompressedImage, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
