#!/usr/bin/env python3
# pose_track_sweep_pi.py  –  Pi drives the search sweep (no 2000,2000)

import cv2, math, time, threading, sys
from collections import deque
import mediapipe as mp

sys.path.append("/home/ceylan/Documents/csse4011Project")
from pi_bt.ble_advertiser import BLEAdvertiserThread

# ─── configuration ──────────────────────────────────────────────────
STREAM_URL       = "http://172.20.10.14:81/stream"
MIRRORED         = True
ANGLE_GO,ANGLE_LR=35,25; OUT_GO,OUT_LR=0.70,0.50
KP,KI,KD         = 2.5,0.1,0.2
DEAD,MAX_STEP    = 1,50
LOST_TIMEOUT     = 2.0          # seconds before we declare “lost”
SWEEP_ANGLES     = (-45, 45)    # degrees left / right
SWEEP_DELAY      = 2.0          # seconds between jumps
BUF_LEN,REQUIRED = 6,4
FPS_ALPHA        = 0.2

mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ─── helper classes ────────────────────────────────────────────────
class Camera:
    def __init__(self,url):
        self.cap=cv2.VideoCapture(url)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open stream {url}")
        self.frame=None; self.lock=threading.Lock(); self.stop=False
        threading.Thread(target=self._loop,daemon=True).start()
    def _loop(self):
        while not self.stop:
            ok,f=self.cap.read()
            if ok:
                with self.lock: self.frame=f
    def read(self):
        with self.lock: return None if self.frame is None else self.frame.copy()
    def release(self): self.stop=True; self.cap.release()

class PID:
    def __init__(self,kp,ki,kd,dead,max_step):
        self.kp,self.ki,self.kd = kp,ki,kd
        self.dead,self.max_step = dead,max_step
        self.i=self.prev_err=self.prev_out=0
    def update(self,err):
        if abs(err)<=self.dead: return 0
        self.i += err
        d = err - self.prev_err; self.prev_err = err
        out = self.kp*err + self.ki*self.i + self.kd*d
        if abs(out-self.prev_out) > self.max_step:
            out = self.prev_out + self.max_step*(1 if out>self.prev_out else -1)
        self.prev_out = out
        return out

# ─── MediaPipe helpers ─────────────────────────────────────────────
def body_box(lm,w,h,m=60):
    xs=[p.x for p in lm]; ys=[p.y for p in lm]
    x1=max(0,int(min(xs)*w)-m); y1=max(0,int(min(ys)*h)-m)
    x2=min(w,int(max(xs)*w)+m); y2=min(h,int(max(ys)*h)+m)
    return x1,y1,x2,y2
def horiz_angle(w,s): return min(abs(math.degrees(math.atan2(w.y-s.y,w.x-s.x))),180)
def classify(lm):
    rs,ls=lm[mp_p.PoseLandmark.RIGHT_SHOULDER],lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw,lw=lm[mp_p.PoseLandmark.RIGHT_WRIST], lm[mp_p.PoseLandmark.LEFT_WRIST]
    span=abs(rs.x-ls.x) or 1e-4
    rdx,ldx=rw.x-rs.x,lw.x-ls.x
    r_out_go,l_out_go = abs(rdx)>OUT_GO*span, abs(ldx)>OUT_GO*span
    r_out_lr,l_out_lr = abs(rdx)>OUT_LR*span, abs(ldx)>OUT_LR*span
    r_h_go,l_h_go = horiz_angle(rw,rs)<ANGLE_GO, horiz_angle(lw,ls)<ANGLE_GO
    r_h_lr,l_h_lr = horiz_angle(rw,rs)<ANGLE_LR, horiz_angle(lw,ls)<ANGLE_LR
    if rw.y<rs.y-0.10 and abs(rdx)<0.12 and lw.y>ls.y-0.05: return "STOP"
    if r_h_go and l_h_go and r_out_go and l_out_go: return "GO"
    left  = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right = r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED: left,right = right,left
    if left:  return "LEFT"
    if right: return "RIGHT"
    return "NONE"

# ─── main ──────────────────────────────────────────────────────────
def main():
    cam = Camera(STREAM_URL)
    tx  = BLEAdvertiserThread(); tx.start()

    pid_pan = PID(KP,KI,KD,DEAD,MAX_STEP)
    pid_tlt = PID(KP,KI,KD,DEAD,MAX_STEP)

    last_seen   = time.time()
    sweep_index = 0              # 0 → -45°, 1 → +45°
    last_sweep  = time.time()

    buf=deque(maxlen=BUF_LEN); deb="NONE"
    prev=time.time(); fps_ema=0.0

    screen_w,screen_h=1280,720
    cv2.namedWindow("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Pose + HeadBox", cv2.WND_PROP_FULLSCREEN,
                          cv2.WINDOW_FULLSCREEN)

    with mp_p.Pose(model_complexity=0,min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:
        try:
            while True:
                frame=cam.read()
                if frame is None: time.sleep(.001); continue
                h,w=frame.shape[:2]; img=frame.copy()

                rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB); rgb.flags.writeable=False
                res=pose.process(rgb)

                found=False; cx=w//2; cy=h//2
                if res.pose_landmarks:
                    lm=res.pose_landmarks.landmark
                    buf.append(classify(lm))
                    x1,y1,x2,y2=body_box(lm,w,h)
                    cx,cy=(x1+x2)//2,(y1+y2)//2
                    mp_d.draw_landmarks(img,res.pose_landmarks,mp_p.POSE_CONNECTIONS)
                    cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,255),2)
                    found=True
                    last_seen=time.time()
                if buf:
                    top=max(set(buf),key=buf.count)
                    if buf.count(top)>=REQUIRED: deb=top

                # ---------- choose control output -------------------------
                if found:
                    pan_cmd = pid_pan.update(w//2 - cx)
                    tilt_cmd= pid_tlt.update(h//2 - cy)
                else:
                    # lost: perform Pi-side sweep pan only
                    if time.time()-last_sweep > SWEEP_DELAY:
                        sweep_index ^= 1                      # toggle 0↔1
                        last_sweep = time.time()
                    pan_cmd  = SWEEP_ANGLES[sweep_index]
                    tilt_cmd = 0

                # ---------- BLE payload -----------------------------------
                pan_dir  = 1 if pan_cmd>=0 else 0
                tilt_dir = 1 if tilt_cmd>=0 else 0
                pan_off  = min(abs(int(pan_cmd)),999)
                tilt_off = min(abs(int(tilt_cmd)),999)
                tx.update_pan (pan_dir*1000 + pan_off)
                tx.update_tilt(tilt_dir*1000 + tilt_off)

                # ---------- HUD & window ----------------------------------
                now=time.time(); fps=1/(now-prev); prev=now
                fps_ema=fps if fps_ema==0 else FPS_ALPHA*fps+(1-FPS_ALPHA)*fps_ema
                cv2.putText(img,deb,(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)
                cv2.putText(img,f"{fps_ema:4.1f} FPS",(10,h-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)
                img=cv2.resize(img,(screen_w,screen_h))
                cv2.imshow("Pose + HeadBox",img)
                if cv2.waitKey(1)&0xFF in (27,ord('q')): break

        finally:
            tx.stop(); tx.join()
            cam.release(); cv2.destroyAllWindows()

if __name__=="__main__":
    main()
