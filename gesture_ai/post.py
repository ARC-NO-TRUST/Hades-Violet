#!/usr/bin/env python3
"""
Threaded, distance-robust traffic-gesture detector
  STOP   – right arm up
  GO     – both arms horizontal
  LEFT   – left arm horizontal
  RIGHT  – right arm horizontal
Debounce + big overlay text + FPS.
"""

import cv2, math, time, threading
import mediapipe as mp
from collections import deque

# ───── thresholds & debounce ─────
MIRRORED     = True
ANGLE_TOL_GO = 35
ANGLE_TOL_LR = 25
OUT_FACT_GO  = 0.70
OUT_FACT_LR  = 0.50
BUF_LEN, REQUIRED = 6, 4
FPS_ALPHA    = 0.2
# ──────────────────────────────────

mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose


# ========== threaded camera ==========
class Camera:
    def __init__(self, src=0):
        self.cap   = cv2.VideoCapture(src)
        self.frame = None
        self.lock  = threading.Lock()
        self.stop  = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while not self.stop:
            ok, frame = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = frame

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def release(self):
        self.stop = True
        self.cap.release()


# ========== pose helpers ==========
def horiz_angle(w, s):
    dx, dy = w.x - s.x, w.y - s.y
    deg = abs(math.degrees(math.atan2(dy, dx)))
    return min(deg, 180 - deg)

def classify(lm):
    rs, ls = lm[mp_p.PoseLandmark.RIGHT_SHOULDER], lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw, lw = lm[mp_p.PoseLandmark.RIGHT_WRIST],   lm[mp_p.PoseLandmark.LEFT_WRIST]

    span  = abs(rs.x - ls.x) or 1e-4
    rdx, rdy = rw.x - rs.x, rw.y - rs.y
    ldx, ldy = lw.x - ls.x, lw.y - ls.y

    r_out_go, l_out_go = abs(rdx) > OUT_FACT_GO*span, abs(ldx) > OUT_FACT_GO*span
    r_out_lr, l_out_lr = abs(rdx) > OUT_FACT_LR*span, abs(ldx) > OUT_FACT_LR*span
    r_h_go, l_h_go = horiz_angle(rw, rs) < ANGLE_TOL_GO, horiz_angle(lw, ls) < ANGLE_TOL_GO
    r_h_lr, l_h_lr = horiz_angle(rw, rs) < ANGLE_TOL_LR, horiz_angle(lw, ls) < ANGLE_TOL_LR

    if rw.y < rs.y - .10 and abs(rdx) < .12 and lw.y > ls.y - .05:
        return "STOP"
    if r_h_go and l_h_go and r_out_go and l_out_go:
        return "GO"

    left_out  = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right_out = r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED:
        left_out, right_out = right_out, left_out
    if left_out:  return "LEFT"
    if right_out: return "RIGHT"
    return "NONE"

def bbox_from_landmarks(lm, w, h, m=40):
    xs = [p.x for p in lm]; ys = [p.y for p in lm]
    x1 = max(0, int(min(xs)*w)-m); y1 = max(0, int(min(ys)*h)-m)
    x2 = min(w, int(max(xs)*w)+m); y2 = min(h, int(max(ys)*h)+m)
    return x1,y1,x2,y2


# ========== main ==========
def main():
    cam = Camera(0)
    buf, deb_pose, last_print = deque(maxlen=BUF_LEN), "NONE", "NONE"
    prev, fps_ema = time.time(), 0.0

    with mp_p.Pose(min_detection_confidence=.5,
                   min_tracking_confidence=.5,
                   model_complexity=0) as pose:        # lite model for speed

        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.001)
                continue

            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            res = pose.process(rgb)
            img = frame.copy()

            raw = "NONE"
            if res.pose_landmarks:
                lm = res.pose_landmarks.landmark
                raw = classify(lm)
                mp_d.draw_landmarks(img, res.pose_landmarks, mp_p.POSE_CONNECTIONS)
                x1,y1,x2,y2 = bbox_from_landmarks(lm, w, h)
                cv2.rectangle(img, (x1,y1), (x2,y2), (0,255,255), 3)

            # debounce
            buf.append(raw)
            top = max(set(buf), key=buf.count)
            if buf.count(top) >= REQUIRED:
                deb_pose = top

            # big label
            font, sc, th = cv2.FONT_HERSHEY_SIMPLEX, 2.0, 4
            tw,_ = cv2.getTextSize(deb_pose, font, sc, th)[0]
            cv2.putText(img, deb_pose, ((w-tw)//2, 70), font, sc, (0,0,255), th, cv2.LINE_AA)

            # FPS
            now = time.time(); fps = 1/(now-prev); prev = now
            fps_ema = fps if fps_ema==0 else FPS_ALPHA*fps + (1-FPS_ALPHA)*fps_ema
            cv2.putText(img, f"{fps_ema:5.1f} FPS", (10, h-30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0,255,0), 3)

            if deb_pose != last_print:
                print(">>>", deb_pose); last_print = deb_pose

            cv2.imshow("Threaded Pose Detector", img)
            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                break

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
