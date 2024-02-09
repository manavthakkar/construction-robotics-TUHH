import cv2
import mediapipe as mp

# Initialize MediaPipe Pose model
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# Function to detect landmarks 23 and 24 (left and right hip) from webcam
def detect_hip_landmarks_from_webcam():
    # Open a connection to the webcam (0 represents the default camera)
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        # Read a frame from the webcam
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Convert the frame to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Make detection
        results = pose.process(frame_rgb)

        # Check if any pose is detected
        if results.pose_landmarks:
            # Extract landmarks for left and right hip (landmarks 23 and 24)
            left_hip = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP.value]
            right_hip = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP.value]

            # Print the coordinates of left and right hip
            print("Left Hip (Landmark 23) - x:", left_hip.x, " y:", left_hip.y)
            print("Right Hip (Landmark 24) - x:", right_hip.x, " y:", right_hip.y)

            # Draw landmarks on the frame
            h, w, c = frame.shape
            left_hip_x, left_hip_y = int(left_hip.x * w), int(left_hip.y * h)
            right_hip_x, right_hip_y = int(right_hip.x * w), int(right_hip.y * h)

            cv2.circle(frame, (left_hip_x, left_hip_y), 10, (0, 255, 0), -1)  # Green circle for left hip
            cv2.circle(frame, (right_hip_x, right_hip_y), 10, (0, 255, 0), -1)  # Green circle for right hip

        # Display the frame
        cv2.imshow("Webcam Pose Detection", frame)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

# Call the function for webcam pose detection
detect_hip_landmarks_from_webcam()
