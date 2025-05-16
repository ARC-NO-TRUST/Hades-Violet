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

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = pose.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark

            # Convenience aliases
            rw = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]
            rs = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
            re = landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW]
            lw = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]
            ls = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
            le = landmarks[mp_pose.PoseLandmark.LEFT_ELBOW]

            # STOP Pose - right hand straight up
            if rw.y < rs.y and abs(rw.x - rs.x) < 0.1:
                label = '[STOP] pose detected!'

            # GO Pose - both arms extended outward (same y-level, wrists far apart)
            elif abs(rw.y - rs.y) < 0.15 and abs(lw.y - ls.y) < 0.15 and abs(rw.x - lw.x) > 0.6:
                label = '[GO] pose detected!'

            # LEFT Pose - only left arm extended
            elif abs(lw.y - ls.y) < 0.15 and abs(rw.y - rs.y) > 0.2 and lw.x < ls.x:
                label = '[LEFT] pose detected!'

            # RIGHT Pose - only right arm extended
            elif abs(rw.y - rs.y) < 0.15 and abs(lw.y - ls.y) > 0.2 and rw.x > rs.x:
                label = '[RIGHT] pose detected!'

            else:
                label = None

            if label:
                cv2.putText(image, label, (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                print(label)

            mp_drawing.draw_landmarks(
                image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        cv2.imshow('MediaPipe Pose', image)

        key = cv2.waitKey(5) & 0xFF
        if key == 27 or key == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
