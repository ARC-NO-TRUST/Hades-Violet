"""
Pose-tracking camera controller (Pi side)

- Uses MediaPipe Pose to classify LEFT/RIGHT/GO/STOP gestures.
- Drives pan/tilt servos via a BLE base MCU.
- While the body is lost, tells the MCU to run an autonomous sweep and
  keeps a software counter in lock-step so tracking resumes smoothly.

UPDATED MACROS
──────────────
PWM_PERIOD_USEC = 20000
PAN_LEFT_USEC   = 2600       # full-left pulse
PAN_RIGHT_USEC  =  700       # full-right pulse
(tilt limits left unchanged: 800-1400 µs)
"""

import cv2, math, time, threading, sys
from collections import deque
import mediapipe as mp

sys.path.append("/home/ceylan/Documents/csse4011Project")
from pi_bt.ble_advertiser import BLEAdvertiserThread

# ── Servo-timing macros (from MCU) ──────────
PWM_PERIOD_USEC = 20000

PAN_LEFT_USEC   = 2600   # ← hard left
PAN_RIGHT_USEC  =  700   # → hard right
PAN_STEP_USEC   =   8    # MCU sweeps ±8 µs every 100 ms

TILT_MAX_USEC   = 1400   # look up
TILT_MIN_USEC   =  800   # look down
TILT_STEP_USEC  =   8

TIMER_PERIOD_S  = 0.100  # MCU sweep-timer period

# ── Gesture-classification params ───────────
MIRRORED = True
ANGLE_GO, ANGLE_LR = 35, 25
OUT_GO, OUT_LR     = 0.70, 0.50
BUF_LEN, REQUIRED  = 6, 4
FPS_ALPHA          = 0.2

mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ── Camera grabber ──────────────────────────
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

# ── PID controller ─────────────────────────
class PID:
    def __init__(self, kp, ki, kd, deadband=3, max_change=30):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0
        self.integral   = 0
        self.prev_output = 0
        self.deadband   = deadband
        self.max_change = max_change
    def update(self, error):
        if abs(error) < self.deadband:
            return 0
        self.integral += error
        derivative     = error - self.prev_error
        self.prev_error = error
        raw = self.kp*error + self.ki*self.integral + self.kd*derivative
        delta = raw - self.prev_output
        if abs(delta) > self.max_change:
            raw = self.prev_output + self.max_change*(1 if delta > 0 else -1)
        self.prev_output = raw
        return raw

# ── Software clone of MCU sweep ────────────
class SweepCounter:
    def __init__(self, low, high, step):
        self.low, self.high = low, high
        self.pos   = (low + high)//2
        self.step  = step
        self.dir   = step
    def tick(self):
        self.pos += self.dir
        if self.pos >= self.high or self.pos <= self.low:
            self.dir = -self.dir
        return self.pos
    def centre_offset(self):
        return self.pos - (self.low + self.high)//2

# ── Pose-helper functions ───────────────────
def horiz_angle(w, s):
    deg = abs(math.degrees(math.atan2(w.y - s.y, w.x - s.x)))
    return min(deg, 180 - deg)

def classify(lm):
    """
    RETURN STRING: "LEFT" / "RIGHT" / "GO" / "STOP" / "NONE"
    """
    rs, ls = lm[mp_p.PoseLandmark.RIGHT_SHOULDER], lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw, lw = lm[mp_p.PoseLandmark.RIGHT_WRIST],   lm[mp_p.PoseLandmark.LEFT_WRIST]
    span = abs(rs.x - ls.x) or 1e-4
    rdx, ldx = rw.x - rs.x, lw.x - ls.x
    # outward extent
    r_out_go, l_out_go = abs(rdx) > OUT_GO*span, abs(ldx) > OUT_GO*span
    r_out_lr, l_out_lr = abs(rdx) > OUT_LR*span, abs(ldx) > OUT_LR*span
    # arm-angle to shoulder line
    r_h_go, l_h_go = horiz_angle(rw, rs) < ANGLE_GO, horiz_angle(lw, ls) < ANGLE_GO
    r_h_lr, l_h_lr = horiz_angle(rw, rs) < ANGLE_LR, horiz_angle(lw, ls) < ANGLE_LR
    # special “STOP” (right hand up, left hand down)
    if rw.y < rs.y - 0.10 and abs(rdx) < 0.12 and lw.y > ls.y - 0.05:
        return "STOP"
    # “GO” (both out flat)
    if r_h_go and l_h_go and r_out_go and l_out_go:
        return "GO"
    # LEFT / RIGHT
    left  = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right = r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED:
        left, right = right, left
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
    advertiser = BLEAdvertiserThread(); advertiser.start()

    buf, deb_pose = deque(maxlen=BUF_LEN), "NONE"
    prev, fps_ema = time.time(), 0.0

    pid_pan  = PID(2.5,0.1,0.2, deadband=3, max_change=50)
    pid_tilt = PID(2.5,0.1,0.2, deadband=3, max_change=50)

    pan_counter  = SweepCounter(PAN_RIGHT_USEC, PAN_LEFT_USEC,  PAN_STEP_USEC)
    tilt_counter = SweepCounter(TILT_MIN_USEC,  TILT_MAX_USEC,  TILT_STEP_USEC)

    sweep_active = False
    last_sweep_update = time.time()
    last_seen_time = time.time(); LOST_TIMEOUT = 2.0

    with mp_p.Pose(model_complexity=0,min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:
        try:
            while True:
                frame = cam.read()
                if frame is None:
                    time.sleep(0.001); continue
                h,w = frame.shape[:2]
                rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB); rgb.flags.writeable=False
                res = pose.process(rgb)
                img = frame.copy(); raw="NONE"

                if res.pose_landmarks:                                      # ─ body found
                    if sweep_active:
                        pid_pan.prev_output  = pan_counter.centre_offset()
                        pid_tilt.prev_output = tilt_counter.centre_offset()
                        sweep_active = False
                        print("[INFO] Body re-acquired – back to PID.")
                    last_seen_time = time.time()
                    lm = res.pose_landmarks.landmark; raw = classify(lm)
                    mp_d.draw_landmarks(img,res.pose_landmarks,mp_p.POSE_CONNECTIONS)
                    bx1,by1,bx2,by2,_ = body_box(lm,w,h,60)
                    cx,cy=(bx1+bx2)//2,(by1+by2)//2
                    err_x=w//2-cx; err_y=h//2-cy
                    cv2.rectangle(img,(bx1,by1),(bx2,by2),(0,255,255),3)

                    pan_out  = pid_pan.update(err_x)
                    tilt_out = max(min(pid_tilt.update(err_y),50),-50)

                else:                                                        # ─ body lost
                    if time.time()-last_seen_time > LOST_TIMEOUT:
                        if not sweep_active:
                            advertiser.update_pan(2000)  # “start sweep”
                            advertiser.update_tilt(2000)
                            sweep_active=True
                            print("[INFO] Target lost – sweep mode.")
                        # advance virtual counters every 100 ms
                        now=time.time()
                        while now-last_sweep_update >= TIMER_PERIOD_S:
                            pan_counter.tick(); tilt_counter.tick()
                            last_sweep_update += TIMER_PERIOD_S
                        continue
                    pan_out  = pid_pan.prev_output*0.9
                    tilt_out = max(min(pid_tilt.prev_output*0.9,50),-50)

                # ─ send incremental command ─────────────
                advertiser.update_pan( (1 if pan_out>=0 else 0)*1000 + min(abs(int(pan_out)),999) )
                advertiser.update_tilt( (1 if tilt_out>=0 else 0)*1000 + min(abs(int(tilt_out)),999) )

                # ─ HUD & stats ──────────────────────────
                buf.append(raw); top=max(set(buf),key=buf.count)
                if buf.count(top)>=REQUIRED: deb_pose=top
                cv2.putText(img,deb_pose,(10,20),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
                fps=1/(time.time()-prev); prev=time.time()
                fps_ema=fps if fps_ema==0 else FPS_ALPHA*fps+(1-FPS_ALPHA)*fps_ema
                cv2.putText(img,f"{fps_ema:4.1f} FPS",(10,h-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                cv2.imshow("Pose + HeadBox",cv2.resize(img,(1280,720)))
                if cv2.waitKey(1)&0xFF in (27,ord('q')): break
        finally:
            advertiser.stop(); advertiser.join()
    cam.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
