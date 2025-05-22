#!/usr/bin/env python3
"""
Multithreaded pose-tracking cam:
  - PoseDetectionThread  – runs MediaPipe in the background
  - SweepThread          – handles left-to-right sweeping when no body found
  - Main thread          – state-machine (SWEEPING / TRACKING), GUI, PID pan
Tilt is fixed at +45 deg.
"""

import cv2, math, time, threading, sys
from collections import deque
import mediapipe as mp

sys.path.append("/home/ceylan/Documents/csse4011Project")
from pi_bt.ble_advertiser import BLEAdvertiserThread   # your real BLE advertiser

# ───────── CONFIG ──────────────────────────────────────────────────────────────
STREAM_URL       = "http://172.20.10.14:81/stream"
MIRRORED         = True
ANGLE_GO, ANGLE_LR = 35, 25
OUT_GO, OUT_LR   = 0.70, 0.50
BUF_LEN, REQUIRED= 6,   4
FPS_ALPHA        = 0.2
# ───────────────────────────────────────────────────────────────────────────────

mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ───────── CAMERA CLASS (non-blocking read) ───────────────────────────────────
class Camera:
    def __init__(self, url: str):
        self.cap  = cv2.VideoCapture(url)
        self.lock = threading.Lock()
        self.stop = False
        self.frame = None
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

# ───────── SIMPLE PID ─────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, deadband=3, max_change=30):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.deadband, self.max_change = deadband, max_change
        self.prev_error = self.integral = self.prev_output = 0
    def update(self, err):
        if abs(err) < self.deadband:
            return 0
        self.integral += err
        deriv = err - self.prev_error
        self.prev_error = err
        raw = self.kp*err + self.ki*self.integral + self.kd*deriv
        delta = raw - self.prev_output
        if abs(delta) > self.max_change:
            raw = self.prev_output + self.max_change*(1 if delta>0 else -1)
        self.prev_output = raw
        return raw

# ───────── BLE ADVERTISER (already implemented in your project) ───────────────
# from pi_bt.ble_advertiser import BLEAdvertiserThread  ← imported above

# ───────── THREAD #1 – POSE DETECTION ─────────────────────────────────────────
class PoseDetectionThread(threading.Thread):
    def __init__(self, cam, pose, shared):
        super().__init__(daemon=True)
        self.cam, self.pose, self.shared = cam, pose, shared
    def run(self):
        while not self.shared['stop']:
            frame = self.cam.read()
            if frame is None:
                time.sleep(0.01)
                continue
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); rgb.flags.writeable = False
            result = self.pose.process(rgb)
            with self.shared['lock']:
                self.shared['frame']  = frame
                self.shared['result'] = result
                self.shared['hw']     = (h, w)

# ───────── THREAD #2 – SWEEP MOTION ───────────────────────────────────────────
class SweepThread(threading.Thread):
    def __init__(self, adv, shared):
        super().__init__(daemon=True)
        self.adv, self.shared = adv, shared
        self.angle, self.dir = -45, 1
    def run(self):
        while not self.shared['stop']:
            time.sleep(3.0)                                    # 3 s delay
            with self.shared['lock']:
                if self.shared['state'] != "SWEEPING":
                    continue
                self.angle += self.dir*30                      # 30 deg step
                if self.angle > 45:
                    self.angle, self.dir = 45, -1
                elif self.angle < -45:
                    self.angle, self.dir = -45, 1
                pan_dir  = 1 if self.angle >= 0 else 0
                pan_off  = min(abs(int(self.angle)), 999)
                self.adv.update_pan(pan_dir*1000 + pan_off)
                self.adv.update_tilt(1000)                     # +45° tilt

# ───────── HELPER FUNCS ───────────────────────────────────────────────────────
def horiz_angle(w, s):
    deg = abs(math.degrees(math.atan2(w.y-s.y, w.x-s.x)))
    return min(deg, 180-deg)

def classify(lm):
    rs, ls = lm[mp_p.PoseLandmark.RIGHT_SHOULDER], lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw, lw = lm[mp_p.PoseLandmark.RIGHT_WRIST],  lm[mp_p.PoseLandmark.LEFT_WRIST]
    span = abs(rs.x-ls.x) or 1e-4
    rdx, ldx = rw.x-rs.x, lw.x-ls.x
    r_out_go, l_out_go = abs(rdx)>OUT_GO*span, abs(ldx)>OUT_GO*span
    r_out_lr, l_out_lr = abs(rdx)>OUT_LR*span, abs(ldx)>OUT_LR*span
    r_h_go, l_h_go = horiz_angle(rw,rs)<ANGLE_GO, horiz_angle(lw,ls)<ANGLE_GO
    r_h_lr, l_h_lr = horiz_angle(rw,rs)<ANGLE_LR, horiz_angle(lw,ls)<ANGLE_LR
    if rw.y < rs.y-0.10 and abs(rdx)<0.12 and lw.y > ls.y-0.05: return "STOP"
    if r_h_go and l_h_go and r_out_go and l_out_go:             return "GO"
    left  = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right = r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED: left, right = right, left
    if left:  return "LEFT"
    if right: return "RIGHT"
    return "NONE"

def body_box(lm,w,h,margin=60):
    xs, ys = [p.x for p in lm], [p.y for p in lm]
    x1,y1  = max(0,int(min(xs)*w)-margin), max(0,int(min(ys)*h)-margin)
    x2,y2  = min(w,int(max(xs)*w)+margin), min(h,int(max(ys)*h)+margin)
    return x1,y1,x2,y2, abs(x2-x1)

# ───────── MAIN LOOP (GUI + STATE MACHINE) ───────────────────────────────────
def main():
    cam   = Camera(STREAM_URL)
    adv   = BLEAdvertiserThread(); adv.start()

    shared = { 'frame':None, 'result':None, 'hw':(0,0),
               'state':"SWEEPING", 'stop':False, 'lock':threading.Lock() }

    pid_pan = PID(1.2,0.02,0.1, deadband=3, max_change=30)
    prev_pan_output, last_seen = 0, 0
    buf, deb_pose = deque(maxlen=BUF_LEN), "NONE"
    fps_ema, prev_time = 0, time.time()

    # start worker threads
    with mp_p.Pose(model_complexity=0, min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:
        PoseDetectionThread(cam, pose, shared).start()
        SweepThread(adv, shared).start()

        cv2.namedWindow("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Pose + HeadBox",
                              cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        try:
            while True:
                with shared['lock']:
                    frame, result = shared['frame'], shared['result']
                    h, w          = shared['hw']
                    state         = shared['state']
                if frame is None:
                    time.sleep(0.01); continue

                img = frame.copy(); raw = "NONE"

                if result and result.pose_landmarks:
                    lm = result.pose_landmarks.landmark
                    last_seen = time.time()
                    mp_d.draw_landmarks(img, result.pose_landmarks,
                                        mp_p.POSE_CONNECTIONS)

                    # classification + yellow box
                    raw = classify(lm)
                    bx1,by1,bx2,by2, bw = body_box(lm,w,h,60)
                    cx = (bx1+bx2)//2
                    cv2.rectangle(img,(bx1,by1),(bx2,by2),(0,255,255),3)

                    err_x = w//2 - cx
                    is_stable = (bw > 0.6*w) and (abs(err_x) < 10)

                    # switch to TRACKING if currently sweeping
                    if state == "SWEEPING":
                        with shared['lock']: shared['state'] = state = "TRACKING"
                        print("[STATE] → TRACKING")

                    # PID pan while tracking
                    if state == "TRACKING":
                        pan_out = 0 if is_stable else pid_pan.update(err_x)
                        pan_out = 0.7*pan_out + 0.3*prev_pan_output
                        prev_pan_output = pan_out
                        pan_dir = 1 if pan_out>=0 else 0
                        pan_off = min(abs(int(pan_out)),999)
                        adv.update_pan(pan_dir*1000 + pan_off)
                        adv.update_tilt(1000)               # keep tilt +45°

                else:
                    # no body detected
                    if state == "TRACKING" and time.time()-last_seen > 2.0:
                        with shared['lock']: shared['state'] = "SWEEPING"
                        print("[STATE] → SWEEPING")

                # ── UI and performance overlay ───────────────────────────
                buf.append(raw)
                top = max(set(buf), key=buf.count)
                if buf.count(top) >= REQUIRED: deb_pose = top
                cv2.putText(img, deb_pose, (10,20),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)

                now = time.time()
                fps = 1 / (now-prev_time); prev_time = now
                fps_ema = fps if fps_ema==0 else FPS_ALPHA*fps + (1-FPS_ALPHA)*fps_ema
                cv2.putText(img,f"{fps_ema:4.1f} FPS",(10,h-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

                cv2.imshow("Pose + HeadBox",
                           cv2.resize(img,(1280,720),cv2.INTER_LINEAR))
                if cv2.waitKey(1) & 0xFF in (27,ord('q')):
                    break
        finally:
            shared['stop'] = True
            adv.stop(); adv.join()
            cam.release()
            cv2.destroyAllWindows()

# ───────── RUN ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()
