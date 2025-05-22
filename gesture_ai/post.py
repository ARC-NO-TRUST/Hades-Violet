import cv2
import math
import time
import threading
from collections import deque
import mediapipe as mp
import sys

sys.path.append("/home/ceylan/Documents/csse4011Project")
from pi_bt.ble_advertiser import BLEAdvertiserThread

# ── Configuration ───────────────────────────
MIRRORED = True
ANGLE_GO, ANGLE_LR = 35, 25
OUT_GO, OUT_LR = 0.70, 0.50
BUF_LEN, REQUIRED = 6, 4
FPS_ALPHA = 0.2

mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ── Camera ───────────────────────────
class Camera:
    def __init__(self, stream_url):
        self.cap = cv2.VideoCapture(stream_url)
        self.frame, self.lock, self.stop = None, threading.Lock(), False
        threading.Thread(target=self._loop, daemon=True).start()
    def _loop(self):
        while not self.stop:
            ok, f = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = f
    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()
    def release(self):
        self.stop = True
        self.cap.release()

# ── PID ───────────────────────────
class PID:
    def __init__(self, kp, ki, kd, deadband=1, max_change=30):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.prev_output = 0
        self.deadband = deadband
        self.max_change = max_change

    def update(self, error):
        if abs(error) < self.deadband:
            return 0
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        raw_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        delta = raw_output - self.prev_output
        if abs(delta) > self.max_change:
            raw_output = self.prev_output + self.max_change * (1 if delta > 0 else -1)
        self.prev_output = raw_output
        return raw_output

# ── Sweeping Search Mode ────────────────────
class SweepSearch:
    def __init__(self, pan_range=60, step=2):
        self.pan_range = pan_range
        self.step = step
        self.angle = 0
        self.direction = 1
    def update(self):
        self.angle += self.step * self.direction
        if self.angle >= 360 or self.angle <= 0:
            self.direction *= -1
            self.angle = max(min(self.angle, 359), 0)
        pan = int(self.pan_range * math.cos(math.radians(self.angle)))
        return pan

# ── Pose Helpers ───────────────────────────
def horiz_angle(w, s):
    deg = abs(math.degrees(math.atan2(w.y - s.y, w.x - s.x)))
    return min(deg, 180 - deg)

def classify(lm):
    rs, ls = lm[mp_p.PoseLandmark.RIGHT_SHOULDER], lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw, lw = lm[mp_p.PoseLandmark.RIGHT_WRIST], lm[mp_p.PoseLandmark.LEFT_WRIST]
    span = abs(rs.x - ls.x) or 1e-4
    rdx, ldx = rw.x - rs.x, lw.x - ls.x
    r_out_go, l_out_go = abs(rdx) > OUT_GO * span, abs(ldx) > OUT_GO * span
    r_out_lr, l_out_lr = abs(rdx) > OUT_LR * span, abs(ldx) > OUT_LR * span
    r_h_go, l_h_go = horiz_angle(rw, rs) < ANGLE_GO, horiz_angle(lw, ls) < ANGLE_GO
    r_h_lr, l_h_lr = horiz_angle(rw, rs) < ANGLE_LR, horiz_angle(lw, ls) < ANGLE_LR
    if rw.y < rs.y - 0.10 and abs(rdx) < 0.12 and lw.y > ls.y - 0.05:
        return "STOP"
    if r_h_go and l_h_go and r_out_go and l_out_go:
        return "GO"
    left = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right = r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED:
        left, right = right, left
    if left:
        return "LEFT"
    if right:
        return "RIGHT"
    return "NONE"

def body_box(lm, w, h, margin=60):
    xs = [p.x for p in lm]
    ys = [p.y for p in lm]
    x1 = max(0, int(min(xs) * w) - margin)
    y1 = max(0, int(min(ys) * h) - margin)
    x2 = min(w, int(max(xs) * w) + margin)
    y2 = min(h, int(max(ys) * h) + margin)
    return x1, y1, x2, y2, abs(x2 - x1)

# ── Main App ────────────────────────────────
def main():
    cam = Camera("http://172.20.10.3:81/stream")
    advertiser = BLEAdvertiserThread()
    advertiser.start()

    buf, deb_pose = deque(maxlen=BUF_LEN), "NONE"
    prev, fps_ema = time.time(), 0.0

    screen_w, screen_h = 1280, 720
    cv2.namedWindow("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    pid_pan = PID(2.5, 0.1, 0.2, deadband=1, max_change=50)
    pid_tilt = PID(2.5, 0.1, 0.2, deadband=1, max_change=50)
    sweeper = SweepSearch(pan_range=80, step=2)

    last_seen_time = time.time()
    LOST_TIMEOUT = 2.0
    track_mode = True

    with mp_p.Pose(model_complexity=0, min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:
        try:
            while True:
                frame = cam.read()
                if frame is None:
                    time.sleep(0.001)
                    continue
                h, w = frame.shape[:2]
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); rgb.flags.writeable = False
                res = pose.process(rgb)
                img = frame.copy()
                raw = "NONE"

                if res.pose_landmarks:
                    last_seen_time = time.time()
                    if not track_mode:
                        print("[INFO] Body re-acquired, switching back to tracking mode.")
                    track_mode = True

                    lm = res.pose_landmarks.landmark
                    raw = classify(lm)
                    mp_d.draw_landmarks(img, res.pose_landmarks, mp_p.POSE_CONNECTIONS)

                    bx1, by1, bx2, by2, _ = body_box(lm, w, h, 60)
                    cx, cy = (bx1 + bx2) // 2, (by1 + by2) // 2
                    err_x = w // 2 - cx
                    err_y = h // 2 - cy
                    cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 255, 255), 3)

                    pan_output = pid_pan.update(err_x)
                    tilt_output = pid_tilt.update(err_y)

                else:
                    if time.time() - last_seen_time > LOST_TIMEOUT:
                        if track_mode:
                            print("[INFO] Lost body, sending sweep command to base.")
                        track_mode = False
                        advertiser.update_pan(2000)
                        advertiser.update_tilt(2000)
                        continue  # skip rest of loop while sweeping
                    else:
                        pan_output = pid_pan.prev_output * 0.9
                        tilt_output = pid_tilt.prev_output * 0.9


                pan_dir = 1 if pan_output >= 0 else 0
                pan_off = min(abs(int(pan_output)), 999)
                pan_payload = pan_dir * 1000 + pan_off

                tilt_dir = 1 if tilt_output >= 0 else 0
                tilt_off = min(abs(int(tilt_output)), 999)
                tilt_payload = tilt_dir * 1000 + tilt_off

                advertiser.update_pan(pan_payload)
                advertiser.update_tilt(tilt_payload)

                buf.append(raw)
                top = max(set(buf), key=buf.count)
                if buf.count(top) >= REQUIRED:
                    deb_pose = top

                cv2.putText(img, deb_pose, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                now = time.time()
                fps = 1 / (now - prev)
                prev = now
                fps_ema = fps if fps_ema == 0 else FPS_ALPHA * fps + (1 - FPS_ALPHA) * fps_ema
                cv2.putText(img, f"{fps_ema:4.1f} FPS", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                img = cv2.resize(img, (screen_w, screen_h), interpolation=cv2.INTER_LINEAR)
                cv2.imshow("Pose + HeadBox", img)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break
        finally:
            advertiser.stop()
            advertiser.join()

    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()