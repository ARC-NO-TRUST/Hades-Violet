import cv2
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_pose    = mp.solutions.pose

def detect_pose(landmarks):
    """Return one of STOP, GO, LEFT, RIGHT or NONE."""
    rw, rs = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST],  landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    lw, ls = landmarks[mp_pose.PoseLandmark.LEFT_WRIST],   landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]

    # compute deltas
    rdx, rdy = rw.x - rs.x, rw.y - rs.y
    ldx, ldy = lw.x - ls.x, lw.y - ls.y

    # STOP
    if rdy < -0.1 and abs(rdx) < 0.1:
        return "STOP"
    # GO
    if abs(rdy) < 0.15 and abs(ldy) < 0.15 and abs(rdx - ldx) > 0.6:
        return "GO"
    # LEFT
    if ldx < -0.1 and abs(ldy) < 0.15:
        return "LEFT"
    # RIGHT
    if rdx > 0.1 and abs(rdy) < 0.15:
        return "RIGHT"
    # no match
    return "NONE"

def main():
    cap = cv2.VideoCapture(0)
    last_pose = None        # what we last printed
    displayed_pose = "NONE" # what we overlay

    with mp_pose.Pose(min_detection_confidence=0.5,
                      min_tracking_confidence=0.5) as pose:

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # BGR → RGB → process → back to BGR
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img.flags.writeable = False
            results = pose.process(img)
            img.flags.writeable = True
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            # default if no landmarks
            pose_name = "NONE"
            if results.pose_landmarks:
                mp_drawing.draw_landmarks(
                    img, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

                # detect current pose
                pose_name = detect_pose(results.pose_landmarks.landmark)

            # overlay this pose every frame
            displayed_pose = pose_name
            cv2.putText(img, f"{displayed_pose}", (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # only print when pose actually changes
            if pose_name != last_pose:
                print(f">>> {pose_name}")
                last_pose = pose_name

            cv2.imshow("MediaPipe Pose", img)
            if cv2.waitKey(5) & 0xFF in (27, ord('q')):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
