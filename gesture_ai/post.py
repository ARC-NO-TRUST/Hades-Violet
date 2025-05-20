#!/usr/bin/env python3
import cv2
import math
import time
import threading
from collections import deque
import mediapipe as mp
import sys

# ── Make sure we can import BLEAdvertiserThread ──────────────────────────────
sys.path.append("/Users/ceylan/csse4011/csse4011Project")
from pi_bt.ble_advertiser import BLEAdvertiserThread

# ── Gesture-AI Configuration ─────────────────────────────────────────────────
MIRRORED       = True
ANGLE_GO, ANGLE_LR = 35, 25
OUT_GO, OUT_LR     = 0.70, 0.50
BUF_LEN, REQUIRED  = 6, 4
FPS_ALPHA          = 0.2
HEAD_LEFT_LIMIT    = 0.35
HEAD_RIGHT_LIMIT   = 0.65

mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ── BLE Advertising Throttle ─────────────────────────────────────────────────
ble_send_interval   = 2.0   # seconds between adverts
last_ble_send_time  = 0

# ── Start your advertiser thread once ────────────────────────────────────────
ble_thread = BLEAdvertiserThread()
ble_thread.start()

# ── Camera Capture ───────────────────────────────────────────────────────────
class Camera:
    def __init__(self, stream_url):
        self.cap   = cv2.VideoCapture(stream_url)
        self.frame = None
        self.lock  = threading.Lock()
        self.stop  = False
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

# ── PID Controller ───────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, deadband=15, max_change=4):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error  = 0
        self.integral    = 0
        self.prev_output = 0
        self.deadband    = deadband
        self.max_change  = max_change

    def update(self, error):
        if abs(error) < self.deadband:
            return 0
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        raw = self.kp*error + self.ki*self.integral + self.kd*derivative
        delta = raw - self.prev_output
        if abs(delta) > self.max_change:
            raw = self.prev_output + self.max_change * (1 if delta>0 else -1)
        self.prev_output = raw
        return raw

# ── Sweep-Search Mode ─────────────────────────────────────────────────────────
class SweepSearch:
    def __init__(self, pan_range=60, tilt_range=30, step=2):
        self.pan_range  = pan_range
        self.tilt_range = tilt_range
        self.step       = step
        self.angle      = 0

    def update(self):
        self.angle = (self.angle + self.step) % 360
        pan  = int(self.pan_range  * math.cos(math.radians(self.angle)))
        tilt = int(self.tilt_range * math.sin(math.radians(self.angle)))
        return pan, tilt

# ── Encode pan/tilt into 4-digit strings ───────────────────────────────────────
def encode_axis(value, axis):
    mag       = min(abs(int(round(value))), 127)
    direction = 1 if value >= 0 else 0
    # '0XXX' = left/up, '1XXX' = right/down
    return f"{direction}{mag:03d}"

# ── Send to your BLEAdvertiserThread ─────────────────────────────────────────
def advertise_offset(pan_str, tilt_str):
    global last_ble_send_time
    now = time.time()
    if now - last_ble_send_time >= ble_send_interval:
        ble_thread.update_pan(pan_str)
        ble_thread.update_tilt(tilt_str)
        last_ble_send_time = now

# ── Pose Helpers ─────────────────────────────────────────────────────────────
def horiz_angle(w, s):
    deg = abs(math.degrees(math.atan2(w.y-s.y, w.x-s.x)))
    return min(deg, 180-deg)

def classify(lm):
    rs, ls = lm[mp_p.PoseLandmark.RIGHT_SHOULDER], lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw, lw = lm[mp_p.PoseLandmark.RIGHT_WRIST],    lm[mp_p.PoseLandmark.LEFT_WRIST]
    span = abs(rs.x - ls.x) or 1e-4
    rdx, ldx = rw.x-rs.x, lw.x-ls.x
    r_h   = horiz_angle(rw, rs);  l_h   = horiz_angle(lw, ls)
    r_out = abs(rdx)>OUT_GO*span; l_out = abs(ldx)>OUT_GO*span
    if rw.y<rs.y-0.10 and abs(rdx)<0.12 and lw.y>ls.y-0.05:
        return "STOP"
    if r_h<ANGLE_GO and l_h<ANGLE_GO and r_out and l_out:
        return "GO"
    left  = (l_h<ANGLE_LR and abs(ldx)>OUT_LR*span) and not (r_h<ANGLE_LR and abs(rdx)>OUT_LR*span)
    right = (r_h<ANGLE_LR and abs(rdx)>OUT_LR*span) and not (l_h<ANGLE_LR and abs(ldx)>OUT_LR*span)
    if MIRRORED: left, right = right, left
    if left:  return "LEFT"
    if right: return "RIGHT"
    return "NONE"

def body_box(lm, w, h, margin=60):
    xs = [p.x for p in lm];  ys = [p.y for p in lm]
    x1 = max(0,int(min(xs)*w)-margin); y1 = max(0,int(min(ys)*h)-margin)
    x2 = min(w,int(max(xs)*w)+margin); y2 = min(h,int(max(ys)*h)+margin)
    return x1, y1, x2, y2

def head_box(lm, w, h, margin=20):
    idx = list(range(9))
    xs  = [lm[i].x for i in idx];  ys = [lm[i].y for i in idx]
    x1  = max(0,int(min(xs)*w)-margin);  y1 = max(0,int(min(ys)*h)-margin)
    x2  = min(w,int(max(xs)*w)+margin);  y2 = min(h,int(max(ys)*h)+margin)
    return x1, y1, x2, y2, sum(xs)/len(xs)

# ── Main Application ─────────────────────────────────────────────────────────
def main():
    cam     = Camera("http://172.20.10.3:81/stream")
    buf     = deque(maxlen=BUF_LEN)
    deb_pose= "NONE"
    prev, fps_ema = time.time(), 0.0

    # Fullscreen display
    screen_w, screen_h = 1280, 720
    cv2.namedWindow("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    pid_pan  = PID(0.4,0.01,0.2)
    pid_tilt = PID(0.4,0.01,0.2)
    sweeper  = SweepSearch()

    with mp_p.Pose(model_complexity=0,
                   min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:

        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.001)
                continue

            h, w = frame.shape[:2]
            rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res   = pose.process(rgb)
            img   = frame.copy()
            raw   = "NONE"

            if res.pose_landmarks:
                lm = res.pose_landmarks.landmark
                raw = classify(lm)
                mp_d.draw_landmarks(img, res.pose_landmarks, mp_p.POSE_CONNECTIONS)

                bx1,by1,bx2,by2 = body_box(lm, w, h)
                cv2.rectangle(img,(bx1,by1),(bx2,by2),(0,255,255),3)

                cx, cy = (bx1+bx2)//2, (by1+by2)//2
                err_x, err_y = cx - w//2, cy - h//2
                pan_out  = pid_pan.update(err_x)
                tilt_out = pid_tilt.update(err_y)

                # Encode & advertise
                pan_str  = encode_axis(pan_out,  "pan")
                tilt_str = encode_axis(tilt_out, "tilt")
                advertise_offset(pan_str, tilt_str)

                # Head box (optional)
                hx1,hy1,hx2,hy2,_ = head_box(lm, w,h)
                cv2.rectangle(img,(hx1,hy1),(hx2,hy2),(255,0,255),3)

            else:
                # Sweep search
                pan_out, tilt_out = sweeper.update()
                pan_str  = encode_axis(pan_out,  "pan")
                tilt_str = encode_axis(tilt_out, "tilt")
                advertise_offset(pan_str, tilt_str)

            # Debounce gesture label
            buf.append(raw)
            top = max(set(buf), key=buf.count)
            if buf.count(top) >= REQUIRED:
                deb_pose = top

            # Draw gesture & FPS
            cv2.putText(img, deb_pose, (10,20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            now  = time.time()
            fps  = 1/(now-prev);  prev = now
            fps_ema = fps if fps_ema==0 else FPS_ALPHA*fps + (1-FPS_ALPHA)*fps_ema
            cv2.putText(img, f"{fps_ema:4.1f} FPS", (10,h-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            img = cv2.resize(img, (screen_w,screen_h), interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Pose + HeadBox", img)
            if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                break

    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
