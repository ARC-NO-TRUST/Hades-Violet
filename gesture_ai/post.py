import cv2
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

cap = cv2.VideoCapture(0)
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the BGR image to RGB.
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        # Make detection.
        results = pose.process(image)

        # Draw the pose annotation on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            # Get right wrist and right shoulder landmarks
            right_wrist = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]
            right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]

            # Check if right wrist is significantly above right shoulder (STOP pose)
            if right_wrist.y < right_shoulder.y and abs(right_wrist.x - right_shoulder.x) < 0.1:
                cv2.putText(image, '[STOP] pose detected!', (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                print("🛑 STOP pose detected!")

            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        cv2.imshow('MediaPipe Pose', image)

        key = cv2.waitKey(5) & 0xFF
        if key == 27 or key == ord('q'):  # Exit on ESC or 'q'
            break

cap.release()
cv2.destroyAllWindows()
