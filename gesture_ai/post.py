#!/usr/bin/env python3
# pose_track.py  –  uses pi_bt.ble_scanner.BLEScannerThread for feedback

import cv2, math, time, threading, queue, sys
from collections import deque
import mediapipe as mp

# ---------------------------------------------------------------------------
# add your project directory then import the two ready-made BLE helpers
# ---------------------------------------------------------------------------
sys.path.append("/home/ceylan/Documents/csse4011Project")

from pi_bt.ble_advertiser import BLEAdvertiserThread          # sends B1:…
from pi_bt.ble_scanner    import BLEScannerThread             # reads A1:…

# ---------------------------------------------------------------------------
# configuration / tuning
# ---------------------------------------------------------------------------
CAM_STREAM            = "http://172.20.10.3:81/stream"
CENTER_PAN_US         = 1300          # pulse-width centre
CENTER_TILT_US        = 1500

# vision-domain PID
KP, KI, KD            = 2.8, 0.02, 0.4
DEAD_PIX, MAX_STEP_US = 4, 80

# sweep when lost
SWEEP_SPAN, SWEEP_STEP = 120, 20
MISS_TOL, ACQ_FRAMES   = 8, 4

# BLE transmit
BLE_DELTA_MIN, BLE_HOLD_MS = 10, 150

# misc
BUF_LEN, REQUIRED, FPS_ALPHA = 6, 4, 0.2
MIRRORED=True; ANGLE_GO,ANGLE_LR=35,25; OUT_GO,OUT_LR=0.70,0.50
mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ---------------------------------------------------------------------------
# helper classes (Camera, PID, Sweep) – same as before
# ---------------------------------------------------------------------------
class Camera:
    def __init__(self,url):
        self.cap=cv2.VideoCapture(url); self.frame=None
        self.lock=threading.Lock(); self.stop=False
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
        self.kp,self.ki,self.kd=kp,ki,kd; self.dead=dead; self.max_step=max_step
        self.i=self.prev_err=self.prev_out=0
    def prime(self,val): self.prev_out=val
    def update(self,err):
        if abs(err)<=self.dead: return 0
        self.i += err
        d = err - self.prev_err; self.prev_err = err
        out = self.kp*err + self.ki*self.i + self.kd*d
        if abs(out-self.prev_out)>self.max_step:
            out = self.prev_out + self.max_step*(1 if out>self.prev_out else -1)
        self.prev_out = out; return out

class Sweep:
    def __init__(self,span,step): self.span,self.step=span,step; self.pos=-span//2
    def next(self):
        self.pos+=self.step
        if self.pos>=self.span//2 or self.pos<=-self.span//2: self.step=-self.step
        return self.pos

# ---------------------------------------------------------------------------
# MediaPipe helpers (body box + gesture classification)
# ---------------------------------------------------------------------------
def body_box(lm,w,h,m=40):
    xs=[p.x for p in lm]; ys=[p.y for p in lm]
    x1=max(0,int(min(xs)*w)-m); y1=max(0,int(min(ys)*h)-m)
    x2=min(w,int(max(xs)*w)+m); y2=min(h,int(max(ys)*h)+m)
    return x1,y1,x2,y2
def horiz_angle(w,s): return min(abs(math.degrees(math.atan2(w.y-s.y,w.x-s.x))),180)
def classify(lm):
    rs,ls=lm[mp_p.PoseLandmark.RIGHT_SHOULDER],lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw,lw=lm[mp_p.PoseLandmark.RIGHT_WRIST],lm[mp_p.PoseLandmark.LEFT_WRIST]
    span=abs(rs.x-ls.x) or 1e-4
    rdx,ldx=rw.x-rs.x,lw.x-ls.x
    r_out_go,l_out_go=abs(rdx)>OUT_GO*span,abs(ldx)>OUT_GO*span
    r_out_lr,l_out_lr=abs(rdx)>OUT_LR*span,abs(ldx)>OUT_LR*span
    r_h_go,l_h_go=horiz_angle(rw,rs)<ANGLE_GO,horiz_angle(lw,ls)<ANGLE_GO
    r_h_lr,l_h_lr=horiz_angle(rw,rs)<ANGLE_LR,horiz_angle(lw,ls)<ANGLE_LR
    if rw.y<rs.y-0.10 and abs(rdx)<0.12 and lw.y>ls.y-0.05: return "STOP"
    if r_h_go and l_h_go and r_out_go and l_out_go: return "GO"
    left  = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right = r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED: left,right=right,left
    if left:  return "LEFT"
    if right: return "RIGHT"
    return "NONE"

# ---------------------------------------------------------------------------
# main loop
# ---------------------------------------------------------------------------
def main():
    cam = Camera(CAM_STREAM)
    tx  = BLEAdvertiserThread(); tx.start()
    scanner = BLEScannerThread(scan_interval=2.0,pause_interval=0.5)
    scanner.start(); scanner.pause()      # initially paused

    fb_pan=fb_tilt=0
    pid_pan=PID(KP,KI,KD,DEAD_PIX,MAX_STEP_US:=MAX_STEP_US)
    pid_tlt=PID(KP,KI,KD,DEAD_PIX,MAX_STEP_US)
    sweep=Sweep(SWEEP_SPAN,SWEEP_STEP)

    state="SWEEP"; miss=acq=0; last_tx=0; last_p=last_t=0
    buf=deque(maxlen=BUF_LEN); deb="NONE"
    fps_ema=0; prev=time.time()

    with mp_p.Pose(model_complexity=0,min_detection_confidence=.5,min_tracking_confidence=.5) as pose:
        try:
            while True:
                # ------------- feedback queue (only when sweeping) ----------
                if state=="SWEEP":
                    scanner.resume()
                    try:
                        while True:
                            d=scanner.get_queue().get_nowait()
                            if d["name"]=="ACTUATOR" and d["value"].startswith("A1:"):
                                fb_pan,fb_tilt = map(int,d["value"][3:].split(","))
                    except queue.Empty:
                        pass
                else:
                    scanner.pause()

                # ------------- camera & pose -------------------------------
                frame=cam.read()
                if frame is None: time.sleep(.001); continue
                h,w=frame.shape[:2]; img=frame.copy()
                rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB); rgb.flags.writeable=False
                res=pose.process(rgb)
                found=False; cx=w//2; cy=h//2
                if res.pose_landmarks:
                    lm=res.pose_landmarks.landmark
                    buf.append(classify(lm))
                    x1,y1,x2,y2=body_box(lm,w,h,60)
                    cx,cy=(x1+x2)//2,(y1+y2)//2
                    mp_d.draw_landmarks(img,res.pose_landmarks,mp_p.POSE_CONNECTIONS)
                    cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,255),2)
                    found=True

                if buf:
                    top=max(set(buf),key=buf.count)
                    if buf.count(top)>=REQUIRED: deb=top

                # ------------- state transitions ---------------------------
                if state=="SWEEP":
                    if found:
                        acq+=1
                        if acq>=ACQ_FRAMES:
                            state="TRACK"; miss=acq=0
                            pid_pan.prime(fb_pan - CENTER_PAN_US)
                            pid_tlt.prime(fb_tilt- CENTER_TILT_US)
                    else: acq=0
                elif state=="TRACK":
                    if not found:
                        miss+=1
                        if miss>MISS_TOL:
                            state="SWEEP"; sweep.pos=-SWEEP_SPAN//2
                    else: miss=0

                # ------------- command generation -------------------------
                if state=="SWEEP":
                    pan_cmd=sweep.next(); tilt_cmd=0
                else:
                    pan_cmd=pid_pan.update(w//2 - cx)
                    tilt_cmd=pid_tlt.update(h//2 - cy)

                pan_dir=1 if pan_cmd>=0 else 0
                tilt_dir=1 if tilt_cmd>=0 else 0
                pan_us=pan_dir*1000 + min(abs(int(pan_cmd)),999)
                tilt_us=tilt_dir*1000+ min(abs(int(tilt_cmd)),999)

                now=time.time()
                if abs(pan_us-last_p)>=BLE_DELTA_MIN or abs(tilt_us-last_t)>=BLE_DELTA_MIN or (now-last_tx)*1000>BLE_HOLD_MS:
                    last_tx=now; last_p,last_t=pan_us,tilt_us
                    tx.update_pan(pan_us); tx.update_tilt(tilt_us)

                # ------------- HUD and window -----------------------------
                fps=1/(now-prev); prev=now
                fps_ema=fps if fps_ema==0 else FPS_ALPHA*fps+(1-FPS_ALPHA)*fps_ema
                cv2.putText(img,deb,(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)
                cv2.putText(img,f"{fps_ema:4.1f} FPS",(10,h-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)
                cv2.imshow("PoseTrack",cv2.resize(img,(1280,720)))
                if cv2.waitKey(1)&0xFF in (27,ord('q')): break
        finally:
            tx.stop(); tx.join()
            scanner.stop(); scanner.join()
            cam.release(); cv2.destroyAllWindows()

if __name__=="__main__":
    main()
