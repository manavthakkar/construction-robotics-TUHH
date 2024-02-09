#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from opencat import roscat
from std_msgs.msg import String
from std_msgs.msg import Bool
from opencat.roscat import Command

#Laptop code
# Initialize MediaPipe Holistic
mp_holistic = mp.solutions.holistic
holistic = mp_holistic.Holistic()

# Initialize MediaPipe Pose model
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# Add a global variable to track the current motion state
current_motion_state = None

status = None

def image_callback(event):
    global current_motion_state

    try:
        
        # Capture video from the laptop camera
        cap = cv2.VideoCapture(0)

        while not rospy.is_shutdown():
            ret, cv_image = cap.read()
            cv_image = cv2.flip(cv_image, 1)
            # Convert the BGR image to RGB
            rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            left = 210
            right = 430
            top = 200
            bottom = 300

            # Draw a rectangle on the frame
            cv2.rectangle(cv_image, (left, top), (right, bottom), (0, 255, 255), 2)

            # Process the frame with MediaPipe Holistic
            results_pose = pose.process(rgb_frame)

            # Check if any pose is detected for Pose model
            if results_pose.pose_landmarks:
                # Extract landmarks for left and right hip (landmarks 23 and 24)
                left_hip = results_pose.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP.value]
                right_hip = results_pose.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP.value]

                # Calculate the midpoint between left and right hip
                midpoint_x = int((left_hip.x + right_hip.x) / 2 * cv_image.shape[1])
                midpoint_y = int((left_hip.y + right_hip.y) / 2 * cv_image.shape[0]) - 40

                h, w, c = cv_image.shape
                left_hip_x, left_hip_y = int(left_hip.x * w), int(left_hip.y * h)
                right_hip_x, right_hip_y = int(right_hip.x * w), int(right_hip.y * h)

                # Draw landmarks and text on the frame
                cv2.circle(cv_image, (midpoint_x, midpoint_y), 10, (255, 0, 0), -1)  # Blue circle for midpoint
                cv2.circle(cv_image, (left_hip_x, left_hip_y), 10, (0, 255, 0), -1)  # Green circle for left hip
                cv2.circle(cv_image, (right_hip_x, right_hip_y), 10, (0, 255, 0), -1)  # Green circle for right hip

                # Print the coordinates of left and right hip and the midpoint
                print("Midpoint - x:", midpoint_x, " y:", midpoint_y)

                if status == 'True':

                    new_motion_state = None
                    
                    if midpoint_x < left:
                        # new_motion_state = roscat.Command.CRAWL_RIGHT
                        new_motion_state  = "left"
                    elif midpoint_x > right:
                        # new_motion_state = roscat.Command.CRAWL_LEFT
                        new_motion_state  = "right"
                    elif midpoint_y < top:
                        # new_motion_state = roscat.Command.BACK
                        new_motion_state  = "up"
                    elif midpoint_y > bottom:
                        # new_motion_state = roscat.Command.WALK
                        new_motion_state  = "down"
                    else:
                        # new_motion_state = roscat.Command.BALANCE
                        new_motion_state  = "STOP"

                    # Only send a new command if the motion state has changed
                    if new_motion_state != current_motion_state:
                        # sc.SendTask(roscat.Task(new_motion_state, [], 2))
                        pub.publish(new_motion_state)
                        current_motion_state = new_motion_state

            # Display the resulting frame
            cv2.imshow("MediaPipe Pose", cv_image)
            cv2.waitKey(1)

    except Exception as e:
        print(e)

    finally:
        cap.release()
        cv2.destroyAllWindows()

def status_callback(data):
    global status
    status= str(data.data)
    rospy.loginfo("Data is " + data)


def main():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Timer(rospy.Duration(0.1), image_callback)  # Use a timer instead of a subscriber for continuous video capture
    global pub
    pub = rospy.Publisher('/motion', String, queue_size=10)
    rate = rospy.Rate(10)

    rospy.Subscriber("/status", Bool, status_callback)

    rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        rospy.logerr("Need robot name")
        sys.exit(1)

    robot_name = str(sys.argv[1])
    sc = roscat.ServiceClient(robot_name)
    main()