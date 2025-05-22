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

class PID:
    def __init__(self, kp, ki, kd, deadband=3, max_change=30):
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

def main():
    cam = Camera("http://172.20.10.14:81/stream")
    advertiser = BLEAdvertiserThread()
    advertiser.start()

    buf, deb_pose = deque(maxlen=BUF_LEN), "NONE"
    prev, fps_ema = time.time(), 0.0

    screen_w, screen_h = 1280, 720
    cv2.namedWindow("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    pid_pan = PID(kp=1.2, ki=0.02, kd=0.1, deadband=3, max_change=30)

    last_seen_time = 0
    prev_pan_output = 0
    state = "SWEEPING"
    sweep_angle = -45
    sweep_direction = 1
    last_sweep_time = time.time()

    with mp_p.Pose(model_complexity=0, min_detection_confidence=.5, min_tracking_confidence=.5) as pose:
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

                person_found = False
                if res.pose_landmarks:
                    person_found = True
                    last_seen_time = time.time()
                    lm = res.pose_landmarks.landmark
                    raw = classify(lm)
                    mp_d.draw_landmarks(img, res.pose_landmarks, mp_p.POSE_CONNECTIONS)

                    bx1, by1, bx2, by2, box_width = body_box(lm, w, h, 60)
                    cx, cy = (bx1 + bx2) // 2, (by1 + by2) // 2
                    err_x = w // 2 - cx
                    err_y = h // 2 - cy
                    cv2.rectangle(img, (bx1, by1), (bx2, by2), (0, 255, 255), 3)  # Yellow box

                    is_stable = (box_width > 0.6 * w) and (abs(err_x) < 10)

                    if state == "TRACKING":
                        if not is_stable:
                            pan_output = pid_pan.update(err_x)
                        else:
                            pan_output = 0
                        pan_output = 0.7 * pan_output + 0.3 * prev_pan_output
                        prev_pan_output = pan_output

                        pan_dir = 1 if pan_output >= 0 else 0
                        pan_off = min(abs(int(pan_output)), 999)
                        pan_payload = pan_dir * 1000 + pan_off
                        advertiser.update_pan(pan_payload)

                    # Change to TRACKING if in sweep
                    if state == "SWEEPING":
                        print("[STATE] Switching to TRACKING")
                        state = "TRACKING"

                else:
                    if state == "TRACKING" and time.time() - last_seen_time > 2.0:
                        print("[STATE] Lost target, switching to SWEEPING")
                        state = "SWEEPING"
                        sweep_angle = -45
                        sweep_direction = 1
                        last_sweep_time = time.time()

                # Handle sweeping logic
                if state == "SWEEPING":
                    if time.time() - last_sweep_time >= 3.0:
                        sweep_angle += sweep_direction * 30
                        if sweep_angle > 45:
                            sweep_angle = 45
                            sweep_direction = -1
                        elif sweep_angle < -45:
                            sweep_angle = -45
                            sweep_direction = 1

                        # Convert to pan payload
                        pan_dir = 1 if sweep_angle >= 0 else 0
                        pan_off = min(abs(int(sweep_angle)), 999)
                        pan_payload = pan_dir * 1000 + pan_off
                        advertiser.update_pan(pan_payload)
                        last_sweep_time = time.time()

                # Fixed tilt at +45°
                advertiser.update_tilt(1000)

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

