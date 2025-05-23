#!/usr/bin/env python3
# pose_track_pi.py – recentres on lost target (non-blocking version)

import cv2, math, time, threading, sys
from collections import deque
import mediapipe as mp
from queue import Empty

sys.path.append("/home/ceylan/Documents/csse4011Project")
from pi_bt.ble_advertiser import BLEAdvertiserThread
from pi_bt.ble_scanner import BLEScannerThread

# ─── Tunables ─────────────────────────────────────────────────────
MIRRORED          = True
ANGLE_GO,ANGLE_LR = 35, 25
OUT_GO, OUT_LR    = 0.70, 0.50
BUF_LEN, REQUIRED = 6, 4
FPS_ALPHA         = 0.2
LOST_TIMEOUT      = 5.0            # s before we declare “lost”

SPECIAL_CMD       = 2000           # Pi→MCU:  centre & hold

# ─── MediaPipe helpers ────────────────────────────────────────────
mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ─── Camera thread ────────────────────────────────────────────────
class Camera:
    def __init__(self, url):
        self.cap   = cv2.VideoCapture(url)
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

# ─── PID ──────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, deadband=3, max_change=30):
        self.kp,self.ki,self.kd = kp,ki,kd
        self.prev_err = 0
        self.integral = 0
        self.prev_out = 0
        self.deadband,self.max_change = deadband,max_change
    def reset(self):
        self.prev_err = self.integral = self.prev_out = 0
    def update(self, err):
        if abs(err) < self.deadband:
            self.integral = 0
            return 0
        self.integral += err
        d       = err - self.prev_err
        self.prev_err = err
        out     = self.kp*err + self.ki*self.integral + self.kd*d
        delta   = out - self.prev_out
        if abs(delta) > self.max_change:
            out = self.prev_out + self.max_change*(1 if delta>0 else -1)
        self.prev_out = out
        return out

# ─── Gesture helpers (same as before) ─────────────────────────────
def horiz_angle(w,s):
    deg = abs(math.degrees(math.atan2(w.y - s.y, w.x - s.x)))
    return min(deg, 180-deg)

def classify(lm):
    rs, ls = lm[mp_p.PoseLandmark.RIGHT_SHOULDER], lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw, lw = lm[mp_p.PoseLandmark.RIGHT_WRIST],    lm[mp_p.PoseLandmark.LEFT_WRIST]
    span   = abs(rs.x - ls.x) or 1e-4
    rdx, ldx = rw.x - rs.x, lw.x - ls.x
    r_out_go, l_out_go = abs(rdx) > OUT_GO*span, abs(ldx) > OUT_GO*span
    r_out_lr, l_out_lr = abs(rdx) > OUT_LR*span, abs(ldx) > OUT_LR*span
    r_h_go,  l_h_go  = horiz_angle(rw, rs) < ANGLE_GO, horiz_angle(lw, ls) < ANGLE_GO
    r_h_lr,  l_h_lr  = horiz_angle(rw, rs) < ANGLE_LR, horiz_angle(lw, ls) < ANGLE_LR
    if rw.y < rs.y - .10 and abs(rdx) < .12 and lw.y > ls.y - .05:
        return "STOP"
    if r_h_go and l_h_go and r_out_go and l_out_go:
        return "GO"
    left  = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right = r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED: left, right = right, left
    if left:  return "LEFT"
    if right: return "RIGHT"
    return "NONE"

def body_box(lm,w,h,m=60):
    xs,ys=[p.x for p in lm],[p.y for p in lm]
    x1=max(0,int(min(xs)*w)-m); y1=max(0,int(min(ys)*h)-m)
    x2=min(w,int(max(xs)*w)+m); y2=min(h,int(max(ys)*h)+m)
    return x1,y1,x2,y2

def extract_ultrasonic(payload):
    """Extracts float distance from 'U1:<int>.<decimal>'."""
    if payload.startswith("U1:"):
        try:
            return float(payload[3:].strip())
        except ValueError:
            pass
    return None

def extract_accel(payload):
    """Extracts x, y, z from 'M1:x,y,z'."""
    if payload.startswith("M1:"):
        try:
            parts = payload[3:].split(",")
            if len(parts) != 3:
                return None
            x, y, z = map(int, parts)
            return {"x": x, "y": y, "z": z}
        except ValueError:
            pass
    return None

# ─── Main loop ────────────────────────────────────────────────────
def main():
    cam = Camera("http://172.20.10.3:81/stream")
    advertiser = BLEAdvertiserThread()
    advertiser.start()
    scanner = BLEScannerThread()
    scanner.start()
    scanner_queue = scanner.get_queue()

    pid_pan  = PID(2.5,.1,.2, deadband=3, max_change=50)
    pid_tilt = PID(2.5,.1,.2, deadband=3, max_change=50)

    buf, deb_pose = deque(maxlen=BUF_LEN), "NONE"
    last_seen = time.time()
    lost_active = False            # NEW
    fps_ema, prev_ts = 0.0, time.time()

    cv2.namedWindow("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN,
                          cv2.WINDOW_FULLSCREEN)

    accel_pos = {"x": 0, "y": 0, "z":0}

    with mp_p.Pose(model_complexity=0,
                   min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:
        try:
            while True:
                f = cam.read()
                if f is None:
                    time.sleep(.001); continue
                h,w = f.shape[:2]
                rgb = cv2.cvtColor(f,cv2.COLOR_BGR2RGB); rgb.flags.writeable=False
                res = pose.process(rgb)
                img = f.copy(); raw="NONE"

                if res.pose_landmarks:                     # ▸ body visible
                    if lost_active:
                        lost_active=False                   # regained
                    last_seen=time.time()
                    lm=res.pose_landmarks.landmark
                    raw=classify(lm)
                    mp_d.draw_landmarks(img,res.pose_landmarks,mp_p.POSE_CONNECTIONS)

                    x1,y1,x2,y2 = body_box(lm,w,h); cx,cy = (x1+x2)//2,(y1+y2)//2
                    cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,255),3)

                    err_x,err_y = w//2 - cx, h//2 - cy
                    pan  = pid_pan.update(err_x)
                    tilt = max(min(pid_tilt.update(err_y),50),-50)

                    advertiser.update_pan( (1 if pan>=0 else 0)*1000 + min(abs(int(pan)),999) )
                    advertiser.update_tilt((1 if tilt>=0 else 0)*1000 + min(abs(int(tilt)),999))

                else:                                       # ▸ body missing
                    if not lost_active and time.time()-last_seen > LOST_TIMEOUT:
                        # enter lost mode once
                        lost_active=True
                        pid_pan.reset(); pid_tilt.reset()
                        advertiser.update_pan(SPECIAL_CMD)
                        advertiser.update_tilt(SPECIAL_CMD)

                    # while lost_active we merely show video; no BLE traffic

                # ── HUD ──────────────────────────────────
                buf.append(raw)
                top=max(set(buf), key=buf.count)
                if buf.count(top)>=REQUIRED: deb_pose=top
                cv2.putText(img,deb_pose,(10,20),
                            cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)

                now=time.time(); fps=1/(now-prev_ts); prev_ts=now
                fps_ema = fps if fps_ema==0 else FPS_ALPHA*fps+(1-FPS_ALPHA)*fps_ema
                cv2.putText(img,f"{fps_ema:4.1f} FPS",(10,h-10),
                            cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),2)

                cv2.imshow("Pose + HeadBox", cv2.resize(img,(1280,720)))
                if cv2.waitKey(1) & 0xFF in (27,ord('q')):
                    break

                # BLE Advertising Packet extraction
                try:
                    msg = scanner_queue.get_nowait()
                    name = msg["name"]
                    value = msg["value"]

                    if name == "MOBILE" and value.startswith("M1:"):
                        new_pos = extract_accel(value)
                        if new_pos:
                            accel_pos.update(new_pos)
                            print(f"[SCAN][MOBILE] ACCEL POS - X: {accel_pos['x']}, Y: {accel_pos['y']}, Z: {accel_pos['z']}\n")
                            if abs(accel_pos['x']) > 750:
                                print("STOP!!!")

                            if accel_pos['y'] > 750:
                                print("LEFT!!!")

                            if accel_pos['y'] < -750:
                                print("RIGHT!!!")

                            if accel_pos['z'] > 750:
                                print("GO!!!")

                        else:
                            print("[SCAN][MOBILE] Invalid accelerometer payload:", value)

                    elif name == "ULTRASONIC" and value.startswith("U1:"):
                        distance = extract_ultrasonic(value)
                        print(f"[SCAN][ULTRASONIC] DISTANCE: {distance} m \n")

                    else:
                        print(f"[SCAN][UNKNOWN] ({name}): {value}\n")

                except Empty:
                    pass
        finally:
            advertiser.stop(); advertiser.join()
            cam.release(); cv2.destroyAllWindows()

if __name__=="__main__":
    main()
