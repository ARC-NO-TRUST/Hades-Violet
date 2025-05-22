#!/usr/bin/env python3
"""
Pose-tracking camera controller (Pi side)  —  FULL SOURCE

- Uses MediaPipe Pose to classify LEFT / RIGHT / GO / STOP gestures.
- Sends pan-tilt commands to a BLE base MCU.
- When the body is lost: orders the MCU to sweep (2000/2000 command),
  advances a software counter in lock-step, and keeps sending a short
  heartbeat so the BLE queue never blocks.

MACROS (match the MCU firmware)
────────────────────────────────
PWM_PERIOD_USEC = 20000
PAN_LEFT_USEC   = 2600        # full-left pulse
PAN_RIGHT_USEC  =  700        # full-right pulse
TILT_MAX_USEC   = 1400        # look up
TILT_MIN_USEC   =  800        # look down
"""

import cv2, math, time, threading, sys
from collections import deque
import mediapipe as mp

sys.path.append("/home/ceylan/Documents/csse4011Project")
from pi_bt.ble_advertiser import BLEAdvertiserThread

# ── Servo-timing macros ─────────────────────
PWM_PERIOD_USEC = 20000        # not used directly here, but kept for reference

PAN_LEFT_USEC   = 2600
PAN_RIGHT_USEC  =  700
PAN_STEP_USEC   =    8         # MCU sweeps ±8 µs every 100 ms

TILT_MAX_USEC   = 1400
TILT_MIN_USEC   =  800
TILT_STEP_USEC  =    8

TIMER_PERIOD_S  = 0.100        # MCU sweep-timer period
SWEEP_HEARTBEAT = 0.500        # resend 2000/2000 every 0.5 s while sweeping

# ── Gesture parameters ──────────────────────
MIRRORED = True                # mirror camera for “selfie” view
ANGLE_GO, ANGLE_LR = 35, 25
OUT_GO, OUT_LR     = 0.70, 0.50
BUF_LEN, REQUIRED  = 6, 4      # debounce gesture classification
FPS_ALPHA          = 0.2       # EMA smoothing for FPS display

mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ── Simple threaded frame grabber ───────────
class Camera:
    def __init__(self, url):
        self.cap = cv2.VideoCapture(url)
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

# ── PID controller ─────────────────────────
class PID:
    def __init__(self, kp, ki, kd, dead=3, max_step=30):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.intg = self.prev_err = self.prev_out = 0
        self.dead, self.max_step  = dead, max_step
    def update(self, err):
        if abs(err) < self.dead:
            return 0
        self.intg += err
        der       = err - self.prev_err
        self.prev_err = err
        raw = self.kp*err + self.ki*self.intg + self.kd*der
        step = raw - self.prev_out
        if abs(step) > self.max_step:
            raw = self.prev_out + self.max_step*(1 if step > 0 else -1)
        self.prev_out = raw
        return raw

# ── Software clone of the MCU sweep ────────
class SweepCounter:
    def __init__(self, low, high, step):
        self.low, self.high, self.step = low, high, step
        self.pos = (low + high)//2
        self.dir = step
    def tick(self):
        self.pos += self.dir
        if self.pos >= self.high or self.pos <= self.low:
            self.dir = -self.dir
        return self.pos
    def centre_offset(self):
        return self.pos - (self.low + self.high)//2

# ── Pose-helper functions ───────────────────
def horiz_angle(w, s):
    """Return acute angle between wrist-shoulder vector and horizontal (deg)."""
    deg = abs(math.degrees(math.atan2(w.y - s.y, w.x - s.x)))
    return min(deg, 180-deg)

def classify(lm):
    """
    Classify LEFT / RIGHT / GO / STOP gestures from landmark list.
    Returns: str in {"LEFT","RIGHT","GO","STOP","NONE"}
    """
    rs, ls = lm[mp_p.PoseLandmark.RIGHT_SHOULDER], lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw, lw = lm[mp_p.PoseLandmark.RIGHT_WRIST],   lm[mp_p.PoseLandmark.LEFT_WRIST]
    span = abs(rs.x - ls.x) or 1e-4
    # horizontal wrist displacement
    rdx, ldx = rw.x - rs.x, lw.x - ls.x
    r_out_go, l_out_go = abs(rdx) > OUT_GO*span, abs(ldx) > OUT_GO*span
    r_out_lr, l_out_lr = abs(rdx) > OUT_LR*span, abs(ldx) > OUT_LR*span
    # angle of arms
    r_h_go, l_h_go = horiz_angle(rw, rs) < ANGLE_GO, horiz_angle(lw, ls) < ANGLE_GO
    r_h_lr, l_h_lr = horiz_angle(rw, rs) < ANGLE_LR, horiz_angle(lw, ls) < ANGLE_LR
    # STOP: right hand raised, left down
    if rw.y < rs.y - 0.10 and abs(rdx) < 0.12 and lw.y > ls.y - 0.05:
        return "STOP"
    # GO: both arms straight out
    if r_h_go and l_h_go and r_out_go and l_out_go:
        return "GO"
    # LEFT / RIGHT
    left  = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right = r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED:
        left, right = right, left   # mirror for selfie view
    if left:  return "LEFT"
    if right: return "RIGHT"
    return "NONE"

def body_box(lm, w, h, margin=60):
    xs=[p.x for p in lm]; ys=[p.y for p in lm]
    x1=max(0,int(min(xs)*w)-margin); y1=max(0,int(min(ys)*h)-margin)
    x2=min(w,int(max(xs)*w)+margin); y2=min(h,int(max(ys)*h)+margin)
    return x1,y1,x2,y2,abs(x2-x1)

# ── Main application loop ───────────────────
def main():
    cam = Camera("http://172.20.10.14:81/stream")
    ble = BLEAdvertiserThread(); ble.start()

    pid_pan  = PID(2.5,0.1,0.2)
    pid_tilt = PID(2.5,0.1,0.2)

    pan_ctr  = SweepCounter(PAN_RIGHT_USEC, PAN_LEFT_USEC,  PAN_STEP_USEC)
    tilt_ctr = SweepCounter(TILT_MIN_USEC,  TILT_MAX_USEC,  TILT_STEP_USEC)

    buf, deb_pose = deque(maxlen=BUF_LEN), "NONE"
    prev, fps_ema = time.time(), 0.0
    last_seen     = time.time()
    sweep_active  = False
    last_tick     = last_heartbeat = time.time()

    # OpenCV full-screen window
    cv2.namedWindow("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Pose + HeadBox",
                          cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    with mp_p.Pose(model_complexity=0,
                   min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:
        try:
            while True:
                frame = cam.read()
                if frame is None:
                    time.sleep(0.005); continue
                h,w = frame.shape[:2]
                rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB); rgb.flags.writeable=False
                res = pose.process(rgb)
                img = frame.copy(); raw = "NONE"

                # ─── BODY FOUND ─────────────────────────
                if res.pose_landmarks:
                    if sweep_active:
                        # re-seed PID with software-predicted servo centre
                        pid_pan.prev_out  = pan_ctr.centre_offset()
                        pid_tilt.prev_out = tilt_ctr.centre_offset()
                        sweep_active = False
                        print("[INFO] body re-acquired → PID tracking")
                    last_seen = time.time()

                    lm = res.pose_landmarks.landmark
                    raw = classify(lm)
                    mp_d.draw_landmarks(img,res.pose_landmarks,
                                        mp_p.POSE_CONNECTIONS, landmark_drawing_spec=None,
                                        connection_drawing_spec=mp_d.DrawingSpec(color=(0,255,0),thickness=2))
                    bx1,by1,bx2,by2,_ = body_box(lm,w,h,60)
                    cx,cy=(bx1+bx2)//2,(by1+by2)//2
                    err_x,err_y = w//2 - cx, h//2 - cy
                    cv2.rectangle(img,(bx1,by1),(bx2,by2),(0,255,255),2)

                    pan_out  = pid_pan.update(err_x)
                    tilt_out = max(min(pid_tilt.update(err_y), 30), -30)

                # ─── BODY LOST ──────────────────────────
                else:
                    dt = time.time() - last_seen
                    if dt > 2.0 and not sweep_active:
                        print("[INFO] target lost → sweep mode")
                        ble.update_pan(2000); ble.update_tilt(2000)
                        sweep_active  = True
                        last_heartbeat = time.time()
                    # advance virtual counters in sync with MCU
                    now = time.time()
                    while now - last_tick >= TIMER_PERIOD_S:
                        pan_ctr.tick(); tilt_ctr.tick()
                        last_tick += TIMER_PERIOD_S

                    # keep PID outputs but dampen
                    pan_out  = pid_pan.prev_out * 0.9
                    tilt_out = max(min(pid_tilt.prev_out*0.9,50), -50)

                # ─── BLE SEND (never blocks) ────────────
                if sweep_active:
                    if time.time() - last_heartbeat >= SWEEP_HEARTBEAT:
                        ble.update_pan(2000); ble.update_tilt(2000)
                        last_heartbeat = time.time()
                else:
                    ble.update_pan( (1 if pan_out>=0 else 0)*1000 + min(abs(int(pan_out)),999) )
                    ble.update_tilt( (1 if tilt_out>=0 else 0)*1000 + min(abs(int(tilt_out)),999) )

                # ─── HUD / stats ───────────────────────
                buf.append(raw)
                leader = max(set(buf), key=buf.count)
                if buf.count(leader) >= REQUIRED:
                    deb_pose = leader
                cv2.putText(img, deb_pose, (10,20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                fps = 1/(time.time()-prev); prev=time.time()
                fps_ema = fps if fps_ema==0 else FPS_ALPHA*fps+(1-FPS_ALPHA)*fps_ema
                cv2.putText(img, f"{fps_ema:4.1f} FPS", (10,h-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                cv2.imshow("Pose + HeadBox",
                           cv2.resize(img,(1280,720)))
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break
        finally:
            ble.stop(); ble.join()
    cam.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
