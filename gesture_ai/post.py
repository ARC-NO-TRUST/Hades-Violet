#!/usr/bin/env python3
"""
Threaded pose detector with head tracking
  - Yellow box = whole body
  - Purple box = head landmarks
  - Big pose label: STOP / GO / LEFT / RIGHT (debounced)
  - When head goes off-centre, prints "MOVE LEFT"/"MOVE RIGHT"
  - FPS overlay
"""

import cv2, math, time, threading
from collections import deque
import mediapipe as mp

# ── pose & debounce thresholds ───────────────────────────
MIRRORED       = True
ANGLE_GO, ANGLE_LR = 35, 25
OUT_GO, OUT_LR     = 0.70, 0.50
BUF_LEN, REQUIRED  = 6, 4
FPS_ALPHA          = 0.2
# Head-centering thresholds (0 … 1, 0.5=center)
HEAD_LEFT_LIMIT  = 0.35
HEAD_RIGHT_LIMIT = 0.65
# ─────────────────────────────────────────────────────────

mp_d, mp_p = mp.solutions.drawing_utils, mp.solutions.pose

# ── threaded camera ─────────────────────────────────────
class Camera:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.frame, self.lock, self.stop = None, threading.Lock(), False
        threading.Thread(target=self._loop, daemon=True).start()
    def _loop(self):
        while not self.stop:
            ok, f = self.cap.read()
            if ok:
                with self.lock: self.frame = f
    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()
    def release(self):
        self.stop = True; self.cap.release()

# ── pose helpers ─────────────────────────────────────────
def horiz_angle(w, s):
    deg = abs(math.degrees(math.atan2(w.y-s.y, w.x-s.x)))
    return min(deg, 180-deg)

def classify(lm):
    rs,ls = lm[mp_p.PoseLandmark.RIGHT_SHOULDER], lm[mp_p.PoseLandmark.LEFT_SHOULDER]
    rw,lw = lm[mp_p.PoseLandmark.RIGHT_WRIST],    lm[mp_p.PoseLandmark.LEFT_WRIST]
    span  = abs(rs.x-ls.x) or 1e-4
    rdx,rdy,ldx,ldy = rw.x-rs.x, rw.y-rs.y, lw.x-ls.x, lw.y-ls.y
    r_out_go,l_out_go = abs(rdx)>OUT_GO*span, abs(ldx)>OUT_GO*span
    r_out_lr,l_out_lr = abs(rdx)>OUT_LR*span, abs(ldx)>OUT_LR*span
    r_h_go,l_h_go = horiz_angle(rw,rs)<ANGLE_GO, horiz_angle(lw,ls)<ANGLE_GO
    r_h_lr,l_h_lr = horiz_angle(rw,rs)<ANGLE_LR, horiz_angle(lw,ls)<ANGLE_LR
    if rw.y < rs.y-.10 and abs(rdx)<.12 and lw.y>ls.y-.05:
        return "STOP"
    if r_h_go and l_h_go and r_out_go and l_out_go:
        return "GO"
    left  = l_h_lr and l_out_lr and not (r_h_lr and r_out_lr)
    right = r_h_lr and r_out_lr and not (l_h_lr and l_out_lr)
    if MIRRORED: left,right = right,left
    if left:  return "LEFT"
    if right: return "RIGHT"
    return "NONE"

def body_box(lm, w, h, margin=60):
    xs=[p.x for p in lm]; ys=[p.y for p in lm]
    x1=max(0,int(min(xs)*w)-margin); y1=max(0,int(min(ys)*h)-margin)
    x2=min(w,int(max(xs)*w)+margin); y2=min(h,int(max(ys)*h)+margin)
    return x1,y1,x2,y2

def head_box(lm, w, h, margin=20):
    idx = [0,1,2,3,4,5,6,7,8]   # nose, eyes, ears
    xs=[lm[i].x for i in idx]; ys=[lm[i].y for i in idx]
    x1=max(0,int(min(xs)*w)-margin); y1=max(0,int(min(ys)*h)-margin)
    x2=min(w,int(max(xs)*w)+margin); y2=min(h,int(max(ys)*h)+margin)
    return x1,y1,x2,y2, (sum(xs)/len(xs))

# ── main loop ────────────────────────────────────────────
def main():
    cam = Camera(0)
    buf, deb_pose, last_print = deque(maxlen=BUF_LEN), "NONE", "NONE"
    prev, fps_ema = time.time(), 0.0
    last_move = "CENTER"

    with mp_p.Pose(model_complexity=0, min_detection_confidence=.5,
                   min_tracking_confidence=.5) as pose:

        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.001); continue
            h,w = frame.shape[:2]

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); rgb.flags.writeable=False
            res = pose.process(rgb)
            img = frame.copy()
            raw = "NONE"

            if res.pose_landmarks:
                lm = res.pose_landmarks.landmark
                raw = classify(lm)
                mp_d.draw_landmarks(img, res.pose_landmarks, mp_p.POSE_CONNECTIONS)

                # yellow body box
                bx1,by1,bx2,by2 = body_box(lm,w,h,60)
                cv2.rectangle(img,(bx1,by1),(bx2,by2),(0,255,255),3)

                # purple head box
                hx1,hy1,hx2,hy2,head_cx_norm = head_box(lm,w,h,20)
                cv2.rectangle(img,(hx1,hy1),(hx2,hy2),(255,0,255),3)

                # --- servo simulation ---
                if head_cx_norm < HEAD_LEFT_LIMIT:
                    move="MOVE LEFT"
                elif head_cx_norm > HEAD_RIGHT_LIMIT:
                    move="MOVE RIGHT"
                else:
                    move="CENTER"

                if move != last_move:
                    print(move)
                    last_move = move

            # debounce pose
            buf.append(raw)
            top=max(set(buf),key=buf.count)
            if buf.count(top)>=REQUIRED: deb_pose=top

            # big pose label
            font,sc,th=cv2.FONT_HERSHEY_SIMPLEX,2.0,4
            tw,_=cv2.getTextSize(deb_pose,font,sc,th)[0]
            cv2.putText(img,deb_pose,((w-tw)//2,70),font,sc,(0,0,255),th,cv2.LINE_AA)

            # FPS
            now=time.time(); fps=1/(now-prev); prev=now
            fps_ema=fps if fps_ema==0 else FPS_ALPHA*fps+(1-FPS_ALPHA)*fps_ema
            cv2.putText(img,f"{fps_ema:5.1f} FPS",(10,h-30),
                        cv2.FONT_HERSHEY_SIMPLEX,1.3,(0,255,0),3)

            if deb_pose!=last_print:
                print(">>>",deb_pose); last_print=deb_pose

            cv2.imshow("Pose + HeadBox",img)
            if cv2.waitKey(1)&0xFF in (27,ord('q')): break

    cam.release(); cv2.destroyAllWindows()

if __name__=="__main__":
    main()
