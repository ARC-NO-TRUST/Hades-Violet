#!/usr/bin/env python3
import cv2, math, time, threading, sys
from collections import deque
import mediapipe as mp

# ---------------------------------------------------------------- BLE helper --
sys.path.append("/home/ceylan/Documents/csse4011Project")
from pi_bt.ble_advertiser import BLEAdvertiserThread

# ----------------------- global tuning ---------------------------------------
CAM_STREAM   = "http://172.20.10.3:81/stream"

PID_KP, PID_KI, PID_KD = 3.0, 0.02, 0.4
DEADBAND_PIX = 4
MAX_STEP_US  = 80
SERVO_LIMIT  = 600

SWEEP_SPAN   = 120
SWEEP_STEP   = 30
MISS_TOL     = 8
LOCK_FRAMES  = 5

MIRRORED     = True
ANGLE_GO, ANGLE_LR = 35, 25
OUT_GO, OUT_LR     = 0.70, 0.50
BUF_LEN, REQUIRED  = 6, 4
FPS_ALPHA          = 0.2

# -------------------- mp shortcuts -------------------------------------------
mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# -------------------- camera threaded reader ---------------------------------
class Camera:
    def __init__(self,url):
        self.cap = cv2.VideoCapture(url)
        self.frame = None; self.lock = threading.Lock(); self.stop=False
        threading.Thread(target=self._loop,daemon=True).start()
    def _loop(self):
        while not self.stop:
            ok,f=self.cap.read()
            if ok:
                with self.lock: self.frame=f
    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()
    def release(self): self.stop=True; self.cap.release()

# -------------------- PID controller -----------------------------------------
class PID:
    def __init__(self,kp,ki,kd,dead=0,max_step=80,i_clamp=2000):
        self.kp,self.ki,self.kd=kp,ki,kd
        self.dead,self.max_step=dead,max_step
        self.i_clamp=i_clamp
        self.prev_err=self.prev_out=self.i=0
        self.avg=[0,0,0]
    def update(self,err):
        self.avg=[err]+self.avg[:2]
        err=sum(self.avg)//len(self.avg)
        if abs(err)<=self.dead: return 0
        self.i=max(-self.i_clamp,min(self.i_clamp,self.i+err))
        d=err-self.prev_err; self.prev_err=err
        out=self.kp*err+self.ki*self.i+self.kd*d
        delta=out-self.prev_out
        if abs(delta)>self.max_step:
            out=self.prev_out+self.max_step*(1 if delta>0 else -1)
        self.prev_out=out; return out

# -------------------- sweeping pan generator ---------------------------------
class Sweep:
    def __init__(self,span=120,step=30):
        self.span,self.step=span,step
        self.pos=-span//2
    def next(self):
        self.pos+=self.step
        if self.pos>=self.span//2 or self.pos<=-self.span//2:
            self.step=-self.step
        return self.pos

# -------------------- helpers -------------------------------------------------
def horiz_angle(w,s):
    deg=abs(math.degrees(math.atan2(w.y-s.y, w.x-s.x)))
    return min(deg,180-deg)

def classify(lm):
    rs,ls=lm[mp_p.PoseLandmark.RIGHT_SHOULDER], lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw,lw=lm[mp_p.PoseLandmark.RIGHT_WRIST] , lm[mp_p.PoseLandmark.LEFT_WRIST]
    span=abs(rs.x-ls.x) or 1e-4
    rdx,ldx=rw.x-rs.x, lw.x-ls.x
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

def body_box(lm,w,h,margin=40):
    xs=[p.x for p in lm]; ys=[p.y for p in lm]
    x1=max(0,int(min(xs)*w)-margin); y1=max(0,int(min(ys)*h)-margin)
    x2=min(w,int(max(xs)*w)+margin); y2=min(h,int(max(ys)*h)+margin)
    return x1,y1,x2,y2

# ------------------------- main ----------------------------------------------
def main():
    cam=Camera(CAM_STREAM)
    ble=BLEAdvertiserThread(); ble.start()

    pid_pan = PID(PID_KP,PID_KI,PID_KD,DEADBAND_PIX,MAX_STEP_US)
    pid_tilt= PID(PID_KP,PID_KI,PID_KD,DEADBAND_PIX,MAX_STEP_US)
    sweep   = Sweep(SWEEP_SPAN,SWEEP_STEP)

    state="SWEEP"; miss=lock=0; fps_ema=0; prev=time.time()
    buf=deque(maxlen=BUF_LEN); deb_pose="NONE"

    with mp_p.Pose(model_complexity=0,min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:
        while True:
            frame=cam.read()
            if frame is None: time.sleep(.001); continue
            h,w=frame.shape[:2]; img=frame.copy()

            cv2.line(img,(w//2,0),(w//2,h),(255,0,0),1)
            cv2.line(img,(0,h//2),(w,h//2),(255,0,0),1)

            found=False; cx=w//2; cy=h//2; raw="NONE"

            rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB); rgb.flags.writeable=False
            res=pose.process(rgb)
            if res.pose_landmarks:
                lm=res.pose_landmarks.landmark
                raw=classify(lm); buf.append(raw)
                x1,y1,x2,y2=body_box(lm,w,h)
                cx,cy=(x1+x2)//2,(y1+y2)//2
                mp_d.draw_landmarks(img,res.pose_landmarks,mp_p.POSE_CONNECTIONS)
                cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,255),2)
                found=True

            top=max(set(buf),key=buf.count) if buf else "NONE"
            if buf.count(top)>=REQUIRED: deb_pose=top

            # state machine ---------------------------------------------------
            if state=="SWEEP":
                if found: state="TRACK"; miss=0
            elif state=="TRACK":
                if not found:
                    miss+=1
                    if miss>MISS_TOL: state="SWEEP"
                else:
                    if abs(w//2-cx)<=DEADBAND_PIX and abs(h//2-cy)<=DEADBAND_PIX:
                        lock+=1
                        if lock>=LOCK_FRAMES: state="LOCK"
                    else: lock=0; miss=0
            elif state=="LOCK":
                if not found or abs(w//2-cx)>DEADBAND_PIX or abs(h//2-cy)>DEADBAND_PIX:
                    state="TRACK"; lock=0; miss=0

            # control ---------------------------------------------------------
            if state=="SWEEP":
                pan=sweep.next(); tilt=0
            else:
                pan=pid_pan.update(w//2-cx)
                tilt=0 if state=="LOCK" else pid_tilt.update(h//2-cy)

            pan=int(max(min(pan,SERVO_LIMIT),-SERVO_LIMIT))
            tilt=int(max(min(tilt,SERVO_LIMIT),-SERVO_LIMIT))

            ble.update_pan((1 if pan>=0 else 0)*1000+min(abs(pan),999))
            ble.update_tilt((1 if tilt>=0 else 0)*1000+min(abs(tilt),999))

            # HUD -------------------------------------------------------------
            cv2.putText(img,deb_pose,(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
            cv2.putText(img,state   ,(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,0),2)
            fps=1/(time.time()-prev); prev=time.time()
            fps_ema=fps if fps_ema==0 else FPS_ALPHA*fps+(1-FPS_ALPHA)*fps_ema
            cv2.putText(img,f"{fps_ema:4.1f} FPS",(10,h-10),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

            cv2.imshow("PoseTrack",cv2.resize(img,(1280,720)))
            if cv2.waitKey(1)&0xFF in (27,ord('q')): break

    cam.release(); cv2.destroyAllWindows(); ble.stop(); ble.join()

if __name__=="__main__":
    main()
