#!/usr/bin/env python3
import cv2, math, time, threading, sys
from collections import deque
from bluepy.btle import Scanner, DefaultDelegate
import mediapipe as mp

# ───────────── BLE advertiser (already in your project) ────────────────
sys.path.append("/home/ceylan/Documents/csse4011Project")
from pi_bt.ble_advertiser import BLEAdvertiserThread          # sends B1:

# ───────────── BLE feedback listener (A1:<pan>,<tilt>) ────────────────
class _BtDelegate(DefaultDelegate):
    def __init__(self, out_q): super().__init__(); self.q = out_q
    def handleDiscovery(self, dev, isNewDev, isNewData):
        if not isNewData: return
        for _, desc, val in dev.getScanData():
            if desc == "Manufacturer":
                try:
                    txt = bytes.fromhex(val).decode(errors="ignore")
                    if txt.startswith("A1:"):
                        pan, tilt = map(int, txt[3:].split(","))
                        self.q.append((pan, tilt))
                except Exception:
                    pass

class ServoFeedback(threading.Thread):
    """Keeps .pan / .tilt updated with the latest A1:<pan>,<tilt> advert."""
    def __init__(self):
        super().__init__(daemon=True)
        self.pan = 0; self.tilt = 0
        self.q = deque(maxlen=1)
        self.sc = Scanner().withDelegate(_BtDelegate(self.q))
    def run(self):
        while True:
            self.sc.scan(0.6)           # 600-ms passive scan
            while self.q:
                self.pan, self.tilt = self.q.popleft()

# ───────────── constants / tuning ──────────────────────────────────────
CAM_STREAM   = "http://172.20.10.3:81/stream"

# vision PID (pixel → µs)
KP, KI, KD   = 3.0, 0.03, 0.5
DEAD_PIX     = 4
MAX_STEP_US  = 80

# second PID (servo error domain)
SERVO_KP     = 0.8
SERVO_DEAD   = 5
SERVO_STEP   = 30
SERVO_LIM    = 600        # clamp output

# hardware pulse-width centres
PAN_MID      = 1300
TILT_MID     = 1500

# sweep & state machine
SWEEP_SPAN   = 120
SWEEP_STEP   = 20
MISS_TOL     = 8
ACQ_FRAMES   = 4
LOCK_FRAMES  = 5
BLE_HOLD_MS  = 150
BLE_DELTA    = 10
SMOOTH_ALPHA = 0.4

# gesture constants
MIRRORED=True; ANGLE_GO,ANGLE_LR=35,25; OUT_GO,OUT_LR=0.70,0.50
BUF_LEN,REQUIRED=6,4; FPS_ALPHA=0.2

mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ───────────── utility classes ─────────────────────────────────────────
class Camera:
    def __init__(self, url):
        self.cap = cv2.VideoCapture(url)
        self.frame=None; self.lock=threading.Lock(); self.stop=False
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

class PID:
    def __init__(self,kp,ki,kd,dead,max_step,i_clamp=2000):
        self.kp,self.ki,self.kd=kp,ki,kd
        self.dead,self.max_step=dead,max_step
        self.i_clamp=i_clamp
        self.i=self.prev_err=self.prev_out=0
        self.avg=[0,0,0]
    def reset(self): self.i=self.prev_err=self.prev_out=0
    def update(self,err):
        self.avg=[err]+self.avg[:2]; err=sum(self.avg)//len(self.avg)
        if abs(err)<=self.dead: return 0
        self.i=max(-self.i_clamp,min(self.i_clamp,self.i+err))
        d=err-self.prev_err; self.prev_err=err
        out=self.kp*err+self.ki*self.i+self.kd*d
        if abs(out-self.prev_out)>self.max_step:
            out=self.prev_out+self.max_step*(1 if out>self.prev_out else -1)
        self.prev_out=out
        return out

class Sweep:
    def __init__(self,span,step):
        self.span,self.step=span,step
        self.pos=-span//2
    def next(self):
        self.pos+=self.step
        if self.pos>=self.span//2 or self.pos<=-self.span//2:
            self.step=-self.step
        return self.pos

# ───────────── gesture & body box helpers ──────────────────────────────
def horiz_angle(w,s):
    deg=abs(math.degrees(math.atan2(w.y-s.y,w.x-s.x))); return min(deg,180-deg)

def classify(lm):
    rs,ls=lm[mp_p.PoseLandmark.RIGHT_SHOULDER],lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw,lw=lm[mp_p.PoseLandmark.RIGHT_WRIST]  ,lm[mp_p.PoseLandmark.LEFT_WRIST]
    span=abs(rs.x-ls.x) or 1e-4
    rdx,ldx=rw.x-rs.x,lw.x-ls.x
    r_out_go,l_out_go=abs(rdx)>OUT_GO*span,abs(ldx)>OUT_GO*span
    r_out_lr,l_out_lr=abs(rdx)>OUT_LR*span,abs(ldx)>OUT_LR*span
    r_h_go,l_h_go=horiz_angle(rw,rs)<ANGLE_GO,horiz_angle(lw,ls)<ANGLE_GO
    r_h_lr,l_h_lr=horiz_angle(rw,rs)<ANGLE_LR,horiz_angle(lw,ls)<ANGLE_LR
    if rw.y<rs.y-0.10 and abs(rdx)<0.12 and lw.y>ls.y-0.05: return "STOP"
    if r_h_go and l_h_go and r_out_go and l_out_go: return "GO"
    left = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right= r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED: left,right=right,left
    if left: return "LEFT"
    if right:return "RIGHT"
    return "NONE"

def body_box(lm,w,h,m=40):
    xs=[p.x for p in lm]; ys=[p.y for p in lm]
    x1=max(0,int(min(xs)*w)-m); y1=max(0,int(min(ys)*h)-m)
    x2=min(w,int(max(xs)*w)+m); y2=min(h,int(max(ys)*h)+m)
    return x1,y1,x2,y2

# ───────────── main loop ───────────────────────────────────────────────
def main():
    cam = Camera(CAM_STREAM)
    tx  = BLEAdvertiserThread(); tx.start()
    fb  = ServoFeedback();       fb.start()

    pid_visP=PID(KP,KI,KD,DEAD_PIX,MAX_STEP_US)
    pid_visT=PID(KP,KI,KD,DEAD_PIX,MAX_STEP_US)
    pid_serP=PID(SERVO_KP,0,0,SERVO_DEAD,SERVO_STEP)
    pid_serT=PID(SERVO_KP,0,0,SERVO_DEAD,SERVO_STEP)

    sweep=Sweep(SWEEP_SPAN,SWEEP_STEP)

    state="SWEEP"; miss=lock=acq=0
    last_tx=0; last_pan=last_tilt=0
    buf=deque(maxlen=BUF_LEN); deb="NONE"
    fps_ema=0; prev=time.time()

    with mp_p.Pose(model_complexity=0,min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:
        while True:
            frame=cam.read()
            if frame is None: time.sleep(.001); continue
            h,w=frame.shape[:2]; img=frame.copy()

            found=False; cx=w//2; cy=h//2; raw="NONE"
            rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB); rgb.flags.writeable=False
            res=pose.process(rgb)
            if res.pose_landmarks:
                lm=res.pose_landmarks.landmark
                raw=classify(lm); buf.append(raw)
                x1,y1,x2,y2=body_box(lm,w,h,60)
                cx,cy=(x1+x2)//2,(y1+y2)//2
                mp_d.draw_landmarks(img,res.pose_landmarks,mp_p.POSE_CONNECTIONS)
                cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,255),2)
                found=True

            if buf:
                top=max(set(buf),key=buf.count)
                if buf.count(top)>=REQUIRED: deb=top

            # ───── state machine ────────────────────────────
            if state=="SWEEP":
                if found:
                    acq+=1
                    if acq>=ACQ_FRAMES:
                        state="TRACK"; miss=0; acq=0
                        pid_visP.reset(); pid_visT.reset()
                else: acq=0
            elif state=="TRACK":
                if not found:
                    miss+=1
                    if miss>MISS_TOL:
                        state="SWEEP"; sweep.pos=-SWEEP_SPAN//2
                else:
                    if abs(w//2-cx)<=DEAD_PIX and abs(h//2-cy)<=DEAD_PIX:
                        lock+=1
                        if lock>=LOCK_FRAMES: state="LOCK"
                    else: lock=0; miss=0
            elif state=="LOCK":
                if not found or abs(w//2-cx)>DEAD_PIX or abs(h//2-cy)>DEAD_PIX:
                    state="TRACK"; lock=0; miss=0

            # ───── desired pan/tilt (vision PID) ───────────
            if state=="SWEEP":
                d_pan=sweep.next(); d_tilt=0
            else:
                d_pan = pid_visP.update(w//2-cx)
                d_tilt= 0 if state=="LOCK" else pid_visT.update(h//2-cy)
            d_pan = int(max(min(d_pan ,SERVO_LIM),-SERVO_LIM))
            d_tilt= int(max(min(d_tilt,SERVO_LIM),-SERVO_LIM))

            # ───── second PID using feedback ───────────────
            err_pan = d_pan  - (fb.pan  - PAN_MID)
            err_tlt = d_tilt - (fb.tilt - TILT_MID)
            cmd_pan = pid_serP.update(err_pan)
            cmd_tlt= pid_serT.update(err_tlt)

            cmd_pan  = int(SMOOTH_ALPHA*last_pan  + (1-SMOOTH_ALPHA)*cmd_pan)
            cmd_tlt  = int(SMOOTH_ALPHA*last_tilt + (1-SMOOTH_ALPHA)*cmd_tlt)
            cmd_pan  = int(max(min(cmd_pan ,SERVO_LIM),-SERVO_LIM))
            cmd_tlt  = int(max(min(cmd_tlt,SERVO_LIM),-SERVO_LIM))

            # ───── BLE rate-limited send ───────────────────
            now=time.time()
            changed = abs(cmd_pan-last_pan)>=BLE_DELTA or abs(cmd_tlt-last_tilt)>=BLE_DELTA
            if changed or (now-last_tx)*1000>BLE_HOLD_MS:
                last_tx=now; last_pan,last_tilt=cmd_pan,cmd_tlt
                tx.update_pan ((1 if cmd_pan >=0 else 0)*1000+min(abs(cmd_pan ),999))
                tx.update_tilt((1 if cmd_tlt>=0 else 0)*1000+min(abs(cmd_tlt),999))

            # ───── HUD ─────────────────────────────────────
            cv2.putText(img,deb,(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.8,(0,0,255),2)
            fps=1/(now-prev); prev=now
            fps_ema=fps if fps_ema==0 else FPS_ALPHA*fps+(1-FPS_ALPHA)*fps_ema
            cv2.putText(img,f"{fps_ema:4.1f} FPS",(10,h-10),
                        cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)
            cv2.imshow("PoseTrack",cv2.resize(img,(1280,720)))
            if cv2.waitKey(1)&0xFF in (27,ord('q')): break

    cam.release(); cv2.destroyAllWindows()
    tx.stop(); tx.join()    # feedback thread is daemon – ends automatically

if __name__=="__main__":
    main()
