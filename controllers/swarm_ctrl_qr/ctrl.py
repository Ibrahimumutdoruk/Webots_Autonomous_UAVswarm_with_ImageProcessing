"""
ctrl.py - Ana kontrolcu sinifi
"""

import math
import json
import numpy as np

from controller import Robot, Keyboard

from constants import (
    NUM_UAVS, TEAM_ID, MAX_QR, TAKEOFF_ALT, REACH_XY, REACH_Z,
    LAND_ALT, ALIGN_TOL, ALIGN_HOLD, OFFSET, QR_WAIT_MS, BCAST_MS,
    COL_R, QR_SCAN_TIMEOUT_MS, QR_SCAN_ALT, MAX_ALT, CAM_FOV,
    MAX_Z_RATE, EWMA_ALPHA,
    QRX, QRY, HX, HY,
    FNONE, FV, FLINE, FARR, LOC,
    ST_IDLE, ST_TAKEOFF, ST_AA, ST_AG, ST_QR_SCAN,
    ST_HOVER, ST_ALIGN, ST_GOTO, ST_WAIT, ST_MANV,
    ST_RA, ST_RXY, ST_RZ, ST_REJ,
    ST_AYRIL_GOTO, ST_AYRIL_LAND, ST_AYRIL_WAIT,
    ST_AYRIL_RISE, ST_AYRIL_REJ,
    STNAMES, PKT_STATE, PKT_QR,
)
from utils import clamp
from qr_gorev import QRGorev, parse_qr
from qr_reader import try_decode_qr
from color_detect import detect_color
from pid import PID

# ====================== ANA KONTROLCU ======================
class Ctrl:
    def __init__(self):
        self.robot=Robot()
        self.ts=int(self.robot.getBasicTimeStep())
        nm=self.robot.getName()
        self.ID=ord(nm[-1])-ord('1')
        if self.ID<0 or self.ID>2: self.ID=0
        self.pid=PID()
        self.qr_tasks=[QRGorev() for _ in range(MAX_QR)]

        self.imu=self.robot.getDevice("inertial_unit")
        self.gps=self.robot.getDevice("gps")
        self.gyro=self.robot.getDevice("gyro")
        self.cam=None
        try:
            c=self.robot.getDevice("down_camera")
            if c: self.cam=c
        except: pass
        self.em=self.robot.getDevice("emitter")
        self.rx=self.robot.getDevice("receiver")
        self.m1=self.robot.getDevice("m1_motor")
        self.m2=self.robot.getDevice("m2_motor")
        self.m3=self.robot.getDevice("m3_motor")
        self.m4=self.robot.getDevice("m4_motor")

        if self.imu: self.imu.enable(self.ts)
        if self.gps: self.gps.enable(self.ts)
        if self.gyro: self.gyro.enable(self.ts)
        if self.cam:
            cam_ts = self.ts if self.ID==1 else self.ts*4
            self.cam.enable(cam_ts)
            print(f"[{self.ID}] Kamera: {self.cam.getWidth()}x{self.cam.getHeight()}"
                  f" ({cam_ts}ms)")
        if self.em: self.em.setChannel(1); self.em.setRange(200)
        if self.rx: self.rx.enable(self.ts); self.rx.setChannel(1)
        self.kb=Keyboard(); self.kb.enable(self.ts)
        for m in [self.m1,self.m2,self.m3,self.m4]:
            m.setPosition(float('inf'))
            m.setVelocity(-1 if m in [self.m1,self.m3] else 1)

        self.x=0; self.y=0; self.z=0; self.vx=0; self.vy=0
        self.roll=0; self.pitch=0; self.yaw=0; self.yr=0
        self.px=0; self.py=0; self.pt=0
        self.hx=HX[self.ID]; self.hy=HY[self.ID]
        self.sx=self.hx; self.sy=self.hy; self.sset=False
        self.st=ST_IDLE; self.form=FNONE; self.qr=0; self.hdg=0
        self.cx=HX[1]; self.cy=HY[1]; self.gcx=HX[1]; self.gcy=HY[1]
        self.atx=self.hx; self.aty=self.hy
        self.alt_t=TAKEOFF_ALT; self.ra_alt=TAKEOFF_ALT
        self.hms=0; self.ams=0; self.lms=0
        self.prog=0; self.agprog=0
        self.adone=[0]*NUM_UAVS
        self.nbsx=list(HX); self.nbsy=list(HY)
        self.nb=[{}]*NUM_UAVS; self.na=[9999]*NUM_UAVS
        self.manv_p=0; self.manv_r=0; self.manv_ms=3000; self.manv_t=0
        self.qrscan_ms=0; self.qrscan_done=False; self._rtz=-1
        self._ba=0; self._la=0
        self.visited=set()

        self.manual=False
        self.saved_st=ST_IDLE
        self.man_spd=0.5
        self._last_arrow=-1

        self.ay_renk=""
        self.ay_tx=0; self.ay_ty=0
        self.ay_ms=0; self.ay_bek=5000
        self.ay_found=False
        self.ay_cx=0; self.ay_cy=0
        self.ay_lock_set=False
        self.vision_ms=0
        self.cam_cx=0
        self.cam_cy=0
        self.cam_ok=False

        self._color_log_ms=0
        self.sm_tz=None

        if self.ID==0:
            print("="*54)
            print("  Laplacian TEKNOFEST 2026 - QR Suru (v5-FIX15)")
            print("="*54)
            print(f"  YARI-OTONOM (varsayilan):")
            print(f"    T=Kalkis  YUK/ASA=ileri/geri  SOL/SAG=yana kayma")
            print(f"    Q/E=irtifa  V/L/A=formasyon")
            print(f"    0=Otonom QR moduna gec  R=Tum suru RTL")
            print(f"    7=ID0_RTL  8=ID1_RTL  9=ID2_RTL (bireysel)")
            print(f"  OTONOM (O'ya bas):")
            print(f"    1-6=QR sec  V/L/A=form  R=RTL  7/8/9=bireysel")
            print(f"  Team:{TEAM_ID+1}")
            print("="*54)
        print(f"[{self.ID}] Hazir home=({self.hx:.1f},{self.hy:.1f})")

    def ye(self,t):
        e=t-self.yaw
        while e>math.pi: e-=2*math.pi
        while e<-math.pi: e+=2*math.pi
        return e

    def fpos(self,cx,cy,h):
        if self.form==FNONE:
            cs=self.nbsx[1] if self.na[1]<500 else HX[1]
            cy_s=self.nbsy[1] if self.na[1]<500 else HY[1]
            if self.ID==1: cs=self.sx; cy_s=self.sy
            dx=self.sx-cs; dy=self.sy-cy_s
            ch=math.cos(h); sh=math.sin(h)
            ileri=dx*ch+dy*sh; sag=dx*sh-dy*ch
        else:
            sag=LOC[self.form][self.ID][0]
            ileri=LOC[self.form][self.ID][1]
        ch=math.cos(h); sh=math.sin(h)
        return cx+ileri*ch+sag*sh, cy+ileri*sh-sag*ch

    def cpush(self):
        rx=0; ry=0
        for i in range(NUM_UAVS):
            if i==self.ID or self.na[i]>500: continue
            nb=self.nb[i]
            if not nb: continue
            dx=self.x-nb.get("x",0); dy=self.y-nb.get("y",0)
            d=math.sqrt(dx*dx+dy*dy)
            if d<COL_R and d>0.01:
                s=(COL_R-d)/d; rx+=dx*s; ry+=dy*s
        n=math.sqrt(rx*rx+ry*ry)
        if n>0.3: rx=rx/n*0.3; ry=ry/n*0.3
        return rx,ry

    def manv_alt_off(self,pitch_d,roll_d):
        """
        Formasyon eğim irtifa ofseti.
        Her drone kendi slot pozisyonuna göre irtifa farkı alır.
        Hedef: sürünün yan görünümü gerçek eğim açısını yansıtsın.
        Formül: offset = slot_mesafesi * tan(açı_rad)
        CIZGI  roll=15° : ID0=-1.5m, ID1=0, ID2=+1.5m
               her slotun irtifa farkı = 1.5 * tan(15°) = 0.402m
        """
        if self.form<0 or self.form>2: return 0
        sag=LOC[self.form][self.ID][0]
        ileri=LOC[self.form][self.ID][1]
        p_off = (-ileri * math.tan(math.radians(abs(pitch_d)))) if pitch_d!=0 else 0
        r_off = ( sag   * math.tan(math.radians(abs(roll_d))))  if roll_d !=0 else 0
        return p_off+r_off

    def p2w(self, cx_n, cy_n):
        w = self.cam.getWidth() if self.cam else 320
        h = self.cam.getHeight() if self.cam else 240
        tan_h = math.tan(CAM_FOV / 2.0)
        tan_v = tan_h * (h / w)
        cr = math.cos(self.roll); sr = math.sin(self.roll)
        cp = math.cos(self.pitch); sp = math.sin(self.pitch)
        cyw = math.cos(self.yaw); syw = math.sin(self.yaw)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cyw, -syw, 0], [syw, cyw, 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        ray_world = R @ np.array([[-cy_n * tan_v], [-cx_n * tan_h], [-1.0]])
        if ray_world[2, 0] >= -0.01:
            return self.x, self.y
        t = -self.z / ray_world[2, 0]
        calc_x = self.x + t * ray_world[0, 0]
        calc_y = self.y + t * ray_world[1, 0]
        if math.sqrt((calc_x - self.x)**2 + (calc_y - self.y)**2) > 30.0:
            return self.x, self.y
        return calc_x, calc_y

    def drive(self,tx,ty,tz,yrd,dt):
        if self.sm_tz is None:
            self.sm_tz=self.z
        diff=tz-self.sm_tz
        if abs(diff)>MAX_Z_RATE*dt:
            self.sm_tz+=MAX_Z_RATE*dt*(1 if diff>0 else -1)
        else:
            self.sm_tz=tz
        actual_tz=self.sm_tz

        ex=tx-self.x; ey=ty-self.y; d=math.sqrt(ex*ex+ey*ey)
        vxd=0; vyd=0
        if d>0.05:
            spd=clamp(d*0.8,0,0.5)
            c=math.cos(self.yaw); s=math.sin(self.yaw)
            vxd=(ex/d)*spd*c+(ey/d)*spd*s
            vyd=-(ex/d)*spd*s+(ey/d)*spd*c
        rx,ry=self.cpush()
        c=math.cos(self.yaw); s=math.sin(self.yaw)
        vxd+=rx*c+ry*s; vyd+=-rx*s+ry*c
        ac=self.pid.alt(actual_tz,self.z,dt)
        pd,rd=self.pid.vel(vxd,vyd,self.vx,self.vy,dt)
        rc,pc,yc=self.pid.att(rd,pd,self.roll,self.pitch,yrd,self.yr,dt)
        w1,w2,w3,w4=PID.mix(ac,rc,pc,yc)
        self.m1.setVelocity(-w1); self.m2.setVelocity(w2)
        self.m3.setVelocity(-w3); self.m4.setVelocity(w4)

    def moff(self):
        for m in [self.m1,self.m2,self.m3,self.m4]: m.setVelocity(0)

    def mtx(self):
        if not self.em: return
        self.em.send(json.dumps({"type":PKT_STATE,"id":self.ID,
            "x":self.x,"y":self.y,"alt":self.z,"st":self.st,
            "form":self.form,"qr":self.qr,"hdg":self.hdg,
            "cx":self.cx,"cy":self.cy,"sx":self.sx,"sy":self.sy,
            "adone":self.adone[self.ID],"man":self.manual,
            "at":self.alt_t,"prog":self.prog}).encode())

    def bcast_qr(self,qi,js):
        if not self.em: return
        self.em.send(json.dumps({"type":PKT_QR,"id":self.ID,
            "qr_idx":qi,"qr_json":js}).encode())

    def mrx(self):
        if not self.rx: return
        while self.rx.getQueueLength()>0:
            try: raw=self.rx.getString(); p=json.loads(raw)
            except: self.rx.nextPacket(); continue
            self.rx.nextPacket()
            pid=p.get("id",-1)
            if pid<0 or pid>=NUM_UAVS or pid==self.ID: continue

            if p.get("type")==PKT_QR:
                qi=p.get("qr_idx",-1); qj=p.get("qr_json","")
                if 0<=qi<MAX_QR and not self.qr_tasks[qi].valid:
                    r=parse_qr(qj)
                    if r and r.qr_id==qi+1:
                        self.qr_tasks[qi]=r; print(f"[{self.ID}] QR{qi+1} radyodan alindi!")
                continue

            self.nb[pid]=p; self.na[pid]=0
            if p.get("adone"): self.adone[pid]=1
            elif p.get("st") in (ST_ALIGN,ST_RA): self.adone[pid]=0
            self.nbsx[pid]=p.get("sx",HX[pid])
            self.nbsy[pid]=p.get("sy",HY[pid])

            if p.get("st")==ST_QR_SCAN and self.st==ST_GOTO and p.get("qr")==self.qr:
                # cx/cy zaten GOTO'da QRX[qr] olmalı ama garantile
                self.cx=QRX[self.qr]; self.cy=QRY[self.qr]
                self.qrscan_ms=0; self.vision_ms=0; self.st=ST_QR_SCAN
                print(f"[{self.ID}] GOTO->QRSCAN (peer{pid} sync)"); self.mtx()

            if pid==0:
                pf=p.get("form",FNONE)
                if pf!=self.form: self.form=pf
                pman=p.get("man",False)
                ps=p.get("st",ST_IDLE)
                if pman and not self.manual:
                    self.manual=True
                    self.cx=p["cx"]; self.cy=p["cy"]; self.hdg=p["hdg"]
                    self.alt_t=p.get("at",self.alt_t)
                    self.st=ST_HOVER
                elif not pman and self.manual:
                    self.manual=False
                if pman and self.manual and ps not in (ST_RA,ST_RXY,ST_RZ):
                    self.cx=p["cx"]; self.cy=p["cy"]; self.hdg=p["hdg"]
                    self.alt_t=p.get("at",self.alt_t)
                if ps==ST_AA and self.st==ST_TAKEOFF and self.z>=TAKEOFF_ALT-0.3:
                    self.hdg=p["hdg"]; self.qr=p["qr"]
                    self.cx=p["cx"]; self.cy=p["cy"]
                    self.atx=self.hx; self.aty=self.hy; self.ams=0; self.st=ST_AA
                if ps==ST_AG and self.st in (ST_AA,ST_TAKEOFF):
                    self.hdg=p["hdg"]; self.cx=p["cx"]; self.cy=p["cy"]
                    self.ams=0; self.st=ST_AG
                if ps==ST_QR_SCAN and self.st==ST_AG:
                    self.qr=p["qr"]; self.vision_ms=0; self.st=ST_QR_SCAN
                    self.qrscan_ms=0
                if ps in (ST_WAIT,ST_MANV,ST_HOVER) and self.st==ST_QR_SCAN:
                    self.qr=p["qr"]
                    self.st=ST_WAIT if self.qr_tasks[self.qr].valid else ST_HOVER
                    self.hms=0
                if ps==ST_ALIGN and self.st in (ST_HOVER,ST_WAIT,ST_MANV,ST_AG):
                    self.hdg=p["hdg"]; self.qr=p["qr"]
                    self.atx,self.aty=self.fpos(self.cx,self.cy,self.hdg)
                    self.prog=0; self.agprog=0; self.ams=0; self.st=ST_ALIGN
                if ps==ST_GOTO and self.st==ST_ALIGN:
                    self.hdg=p["hdg"]; self.cx=p["cx"]; self.cy=p["cy"]
                # RTL sync: peer ST_RA veya ST_RXY'ye geçince biz de RTL'e katılalım
                if ps in (ST_RA, ST_RXY) and not p.get("man",False) and self.st not in (
                        ST_IDLE,ST_RA,ST_RXY,ST_RZ,
                        ST_AYRIL_GOTO,ST_AYRIL_LAND,ST_AYRIL_WAIT,ST_AYRIL_RISE):
                    self.ra_alt=self.z; self.lms=0; self._rtz=-1; self.ams=0
                    self.st=ST_RA
                    print(f"[{self.ID}] RTL peer sync (peer{pid} st={STNAMES[ps]})")
        for i in range(NUM_UAVS):
            if i!=self.ID: self.na[i]+=self.ts

    def apply_task(self,qi):
        if qi<0 or qi>=MAX_QR: return
        q=self.qr_tasks[qi]
        if not q.valid: self.st=ST_HOVER; return
        self.visited.add(qi)
        print(f"[{self.ID}] === QR{qi+1} GOREV === (ziyaret: {[x+1 for x in self.visited]})")
        if q.form_aktif and q.form_tip!=FNONE and q.form_tip!=self.form:
            self.form=q.form_tip
            print(f"[{self.ID}]  Form->{['V','CIZGI','OKBASI'][self.form]}")
            self.mtx()
        if q.irtifa_aktif:
            self.alt_t=clamp(q.irtifa_m,0.5,MAX_ALT)
            self.sm_tz=None
            print(f"[{self.ID}]  Irtifa->{self.alt_t:.1f}m"); self.mtx()
        if q.ayril_aktif and q.ayril_drone_id==(self.ID+1):
            self.ay_renk=q.ayril_renk
            self.ay_bek=q.ayril_bekle_ms if q.ayril_bekle_ms>0 else 5000
            self.ay_ms=0; self.ay_found=False; self.ay_lock_set=False
            self.ay_cx=self.cx; self.ay_cy=self.cy
            self.st=ST_AYRIL_GOTO
            print(f"[{self.ID}]  >>> AYRILMA -> {self.ay_renk}")
            return

        if q.manv_aktif and (q.pitch_deg != 0 or q.roll_deg != 0):
            self.manv_p = q.pitch_deg
            self.manv_r = q.roll_deg
            self.manv_ms = q.bekle_ms if q.bekle_ms > 0 else 3000
            self.manv_t = 0
            self.sm_tz = self.alt_t
            self.st = ST_MANV
            print(f"[{self.ID}] SÜRÜ EĞİMİ ALINIYOR: p={q.pitch_deg} r={q.roll_deg} ({self.manv_ms/1000}s beklenecek)")
        else:
            self.manv_p = 0; self.manv_r = 0
            # FIX-16: ID0 gecikme düzeltmesi.
            # HOVER'a geçip bir sonraki döngüde ALIGN'a gitmek yerine
            # eğer sonraki QR varsa buradan direkt ALIGN'a geç.
            # Böylece ID1/ID2 ile aynı anda ALIGN'a girilir.
            if self.ID==0:
                nxt = q.sonraki[TEAM_ID] if q.valid else -1
                if nxt>=0 and nxt not in self.visited:
                    AYRIL_STATES = {ST_AYRIL_GOTO, ST_AYRIL_LAND,
                                    ST_AYRIL_WAIT, ST_AYRIL_RISE, ST_AYRIL_REJ}
                    any_ayril = any(
                        self.nb[i].get("st") in AYRIL_STATES
                        for i in range(NUM_UAVS)
                        if i != self.ID and self.na[i] < 2000
                    )
                    if not any_ayril:
                        dx=QRX[nxt]-self.cx; dy=QRY[nxt]-self.cy
                        if math.sqrt(dx*dx+dy*dy)>0.5: self.hdg=math.atan2(dy,dx)
                        self.qr=nxt
                        self.atx,self.aty=self.fpos(self.cx,self.cy,self.hdg)
                        self.ams=0
                        for i in range(NUM_UAVS): self.adone[i]=0
                        self.st=ST_ALIGN
                        print(f"[{self.ID}] ->QR{nxt+1} (apply_task direkt)"); self.mtx()
                        return
            self.st = ST_HOVER

    # ---- RTL ----
    def srtla(self):
        """
        Tüm sürü RTL komutu (R tuşu / görev sonu).
        ST_RA'ya geçer → ID1/ID2 peer sync bunu görür ve RTL'e katılır.
        ST_RA içinde grup bekleme yoktur, hemen ST_RXY'ye devam edilir.
        """
        if self.st in (ST_IDLE,ST_RA,ST_RXY,ST_RZ): return
        self.ra_alt=self.z; self.lms=0; self._rtz=-1
        self.ams=0
        self.st=ST_RA
        print(f"[{self.ID}] RTL -> ST_RA"); self.mtx()

    # ====================== DURUM MAKINESI ======================
    def step_ctrl(self,dt):
        if dt<1e-6: return
        s=self.st

        if s==ST_IDLE: return

        elif s==ST_TAKEOFF:
            self.drive(self.hx,self.hy,TAKEOFF_ALT,0,dt)
            if self.z>=TAKEOFF_ALT-0.1:
                self.cx=self.nbsx[1] if self.na[1]<500 else HX[1]
                self.cy=self.nbsy[1] if self.na[1]<500 else HY[1]
                if self.ID==1: self.cx=self.sx; self.cy=self.sy
                if self.manual:
                    self.st=ST_HOVER; print(f"[{self.ID}] Kalkis->HOVER [YARI-OTONOM]")
                else:
                    dx=QRX[0]-self.cx; dy=QRY[0]-self.cy
                    self.hdg=math.atan2(dy,dx); self.qr=0; self.ams=0
                    self.atx,self.aty=self.fpos(self.cx,self.cy,self.hdg)
                    self.st=ST_AA; print(f"[{self.ID}] Kalkis->AA")

        elif s==ST_AA:
            e=self.ye(self.hdg)
            self.drive(self.atx,self.aty,self.alt_t,clamp(e*3,-1,1),dt)
            if abs(e)<ALIGN_TOL:
                self.ams+=self.ts
                if self.ams>=ALIGN_HOLD:
                    self.gcx=self.cx; self.gcy=self.cy
                    self.cx=QRX[0]; self.cy=QRY[0]
                    self.agprog=0; self.ams=0; self.st=ST_AG
                    print(f"[{self.ID}] AA->AG"); self.mtx()
            else: self.ams=0

        elif s==ST_AG:
            mx=self.cx-self.gcx; my=self.cy-self.gcy
            md=math.sqrt(mx*mx+my*my)
            self.agprog+=0.4*dt
            if self.agprog>=md: self.agprog=md
            f=self.agprog/md if md>0.01 else 0
            tx,ty=self.fpos(self.gcx+mx*f,self.gcy+my*f,self.hdg)
            self.drive(tx, ty, self.alt_t, clamp(self.ye(self.hdg)*2,-0.5,0.5), dt)
            if self.agprog>=md:
                dd=math.sqrt((self.x-tx)**2+(self.y-ty)**2)
                if dd<REACH_XY and abs(self.z-self.alt_t)<REACH_Z:
                    self.agprog=0; self.qrscan_ms=0; self.vision_ms=0
                    self.st=ST_QR_SCAN
                    print(f"[{self.ID}] QR{self.qr+1}->TARAMA"); self.mtx()

        elif s==ST_QR_SCAN:
            # cx/cy kesinlikle QR koordinatı olmalı - garantile
            if abs(self.cx - QRX[self.qr]) > 0.5 or abs(self.cy - QRY[self.qr]) > 0.5:
                self.cx=QRX[self.qr]; self.cy=QRY[self.qr]
            tx,ty=self.fpos(self.cx,self.cy,self.hdg)
            scan_alt = min(self.alt_t, QR_SCAN_ALT)
            self.drive(tx, ty, scan_alt, clamp(self.ye(self.hdg)*2,-0.5,0.5), dt)
            self.qrscan_ms+=self.ts
            if self.qr_tasks[self.qr].valid:
                self.hms=0; self.st=ST_WAIT; return

            # Sadece ID=1 tarar
            if self.ID==1 and self.cam is not None:
                self.vision_ms += self.ts
                if self.vision_ms >= 200:
                    self.vision_ms = 0
                    js=try_decode_qr(self.cam)
                    if js:
                        r=parse_qr(js)
                        if r and r.qr_id == self.qr+1:
                            print(f"[{self.ID}] *** QR{self.qr+1} OKUNDU ***")
                            self.qr_tasks[self.qr]=r
                            self.bcast_qr(self.qr,js)
                            self.bcast_qr(self.qr,js)
                            self.hms=0; self.st=ST_WAIT
                            print(f"[{self.ID}] QR{self.qr+1} yayinlandi"); self.mtx()
                        elif r:
                            print(f"[{self.ID}] QR{r.qr_id} reddedildi (beklenen QR{self.qr+1})")

            if self.qrscan_ms>=QR_SCAN_TIMEOUT_MS and not self.qr_tasks[self.qr].valid:
                print(f"[{self.ID}] QR{self.qr+1} TIMEOUT"); self.st=ST_HOVER

        elif s==ST_HOVER:
            # Ok tuşları burada işlenmez - keys() içinde yönetilir
            tx,ty=self.fpos(self.cx,self.cy,self.hdg)
            yr=clamp(self.ye(self.hdg)*2,-0.5,0.5) if self.manual else 0
            self.drive(tx,ty,self.alt_t,yr,dt)
            if self.manual: return
            if self.ID==0 and any(self.qr_tasks[i].valid for i in range(MAX_QR)):
                q=self.qr_tasks[self.qr]
                nxt=q.sonraki[TEAM_ID] if q.valid else -1
                if nxt>=0 and nxt in self.visited:
                    print(f"[{self.ID}] QR{nxt+1} zaten ziyaret edildi -> GOREV TAMAM -> RTL")
                    nxt=-1
                if nxt==-1:
                    print(f"[{self.ID}] Gorev bitti->RTL"); self.srtla()
                elif nxt!=self.qr:
                    AYRIL_STATES = {ST_AYRIL_GOTO, ST_AYRIL_LAND,
                                    ST_AYRIL_WAIT, ST_AYRIL_RISE, ST_AYRIL_REJ}
                    any_ayril = any(
                        self.nb[i].get("st") in AYRIL_STATES
                        for i in range(NUM_UAVS)
                        if i != self.ID and self.na[i] < 2000
                    )
                    if any_ayril:
                        return
                    dx=QRX[nxt]-self.cx; dy=QRY[nxt]-self.cy
                    if math.sqrt(dx*dx+dy*dy)>0.5: self.hdg=math.atan2(dy,dx)
                    self.qr=nxt
                    self.atx,self.aty=self.fpos(self.cx,self.cy,self.hdg)
                    self.ams=0
                    # FIX-15: adone sifirla ki ALIGN takılmasın
                    for i in range(NUM_UAVS): self.adone[i]=0
                    self.st=ST_ALIGN
                    print(f"[{self.ID}] ->QR{nxt+1}"); self.mtx()

        elif s==ST_ALIGN:
            e=self.ye(self.hdg)
            self.drive(self.atx,self.aty,self.alt_t,clamp(e*3,-1,1),dt)
            if abs(e)<ALIGN_TOL:
                dist_to_slot = math.sqrt((self.x-self.atx)**2+(self.y-self.aty)**2)
                if dist_to_slot < REACH_XY * 4:
                    self.ams+=self.ts
                    if self.ams>=ALIGN_HOLD:
                        self.adone[self.ID]=1; self.ams=0; self.mtx()
                else:
                    self.ams=0
            else: self.ams=0
            if self.adone[self.ID]:
                ok=all(self.adone[i] or i==self.ID or self.na[i]>2000
                       for i in range(NUM_UAVS))
                if ok:
                    self.gcx=self.cx; self.gcy=self.cy
                    self.cx=QRX[self.qr]; self.cy=QRY[self.qr]
                    self.prog=0
                    for i in range(NUM_UAVS): self.adone[i]=0
                    self.st=ST_GOTO
                    print(f"[{self.ID}] Suru->QR{self.qr+1}"); self.mtx()

        elif s==ST_GOTO:
            # cx/cy bu aşamada kesinlikle QR hedefi olmalı (ALIGN->GOTO geçişinde set edilir)
            # Peer sync ile GOTO'ya gelinmişse cx/cy eksik olabilir, garantile
            if abs(self.cx - QRX[self.qr]) > 0.5 or abs(self.cy - QRY[self.qr]) > 0.5:
                self.cx=QRX[self.qr]; self.cy=QRY[self.qr]
            tx,ty=self.fpos(self.cx,self.cy,self.hdg)
            m_off=self.manv_alt_off(self.manv_p,self.manv_r)
            hedef_irtifa=clamp(self.alt_t+m_off,0.5,MAX_ALT)
            self.drive(tx,ty,hedef_irtifa,clamp(self.ye(self.hdg)*2,-0.5,0.5),dt)

            dd=math.sqrt((self.x-tx)**2+(self.y-ty)**2)
            if dd<REACH_XY and abs(self.z-hedef_irtifa)<REACH_Z:
                self.prog=0; self.qrscan_ms=0; self.vision_ms=0
                self.st=ST_QR_SCAN
                print(f"[{self.ID}] QR{self.qr+1}->TARAMA"); self.mtx()

        elif s==ST_WAIT:
            tx,ty=self.fpos(self.cx,self.cy,self.hdg)
            self.drive(tx, ty, self.alt_t, clamp(self.ye(self.hdg)*2,-0.5,0.5), dt)
            self.hms+=self.ts
            bek=self.qr_tasks[self.qr].bekle_ms if self.qr_tasks[self.qr].valid else QR_WAIT_MS
            if bek<=0: bek=QR_WAIT_MS
            if self.hms>=bek: self.apply_task(self.qr)

        elif s==ST_MANV:
            off=self.manv_alt_off(self.manv_p,self.manv_r)
            tgt=clamp(self.alt_t+off, 0.3, MAX_ALT)
            tx,ty=self.fpos(self.cx,self.cy,self.hdg)
            self.drive(tx,ty,tgt,clamp(self.ye(self.hdg)*2,-0.5,0.5),dt)
            if abs(self.z - tgt) < REACH_Z * 2:
                self.manv_t+=self.ts
            if self.manv_t>=self.manv_ms:
                self.st=ST_HOVER; print(f"[{self.ID}] Manevra tamamlandi")

        elif s==ST_AYRIL_GOTO:
            if self.ay_renk.upper()=="MAVI":
                fb_x=15.0; fb_y=0.0
            elif self.ay_renk.upper()=="KIRMIZI":
                fb_x=20.0; fb_y=10.0
            else:
                fb_x=self.cx; fb_y=self.cy

            fb_d=math.sqrt((self.x-fb_x)**2+(self.y-fb_y)**2)

            self.vision_ms += self.ts
            if self.vision_ms >= 250:
                self.vision_ms = 0
                if self.cam is not None:
                    self.cam_cx, self.cam_cy, self.cam_ok = detect_color(self.cam, self.ay_renk)
                    if self.cam_ok and self.ay_found:
                        self._color_log_ms += 250
                        if self._color_log_ms >= 2000:
                            self._color_log_ms = 0
                            wx, wy = self.p2w(self.cam_cx, self.cam_cy)
                            print(f"[{self.ID}] {self.ay_renk} takip | "
                                  f"drone({self.x:.1f},{self.y:.1f},{self.z:.1f}) "
                                  f"-> hedef GPS({wx:.1f},{wy:.1f}) "
                                  f"| kamera norm({self.cam_cx:.2f},{self.cam_cy:.2f})")

            if not self.ay_found:
                self.drive(fb_x,fb_y,self.alt_t,0,dt)
                if self.cam_ok:
                    self.ay_found=True
                    wx,wy=self.p2w(self.cam_cx, self.cam_cy)
                    self.ay_tx=wx; self.ay_ty=wy
                    self.ay_lock_set=True
                    self._color_log_ms=0
                    print(f"[{self.ID}] {'='*48}")
                    print(f"[{self.ID}] *** {self.ay_renk} KAMERA ILE ALGILANDI ***")
                    print(f"[{self.ID}]   Drone GPS      : ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})")
                    print(f"[{self.ID}]   Hedef GPS (p2w): ({wx:.2f}, {wy:.2f})")
                    print(f"[{self.ID}]   Kamera px norm : ({self.cam_cx:.3f}, {self.cam_cy:.3f})")
                    print(f"[{self.ID}] {'='*48}")
                elif fb_d<0.5:
                    self.ay_found=True
                    self.ay_tx=fb_x; self.ay_ty=fb_y
                    self.ay_lock_set=True
                    print(f"[{self.ID}] {self.ay_renk} GPS ile ulasildi -> ({fb_x:.1f},{fb_y:.1f})")
            else:
                if self.cam_ok:
                    wx,wy=self.p2w(self.cam_cx, self.cam_cy)
                    if self.ay_lock_set:
                        self.ay_tx=self.ay_tx*(1-EWMA_ALPHA)+wx*EWMA_ALPHA
                        self.ay_ty=self.ay_ty*(1-EWMA_ALPHA)+wy*EWMA_ALPHA
                    else:
                        self.ay_tx=wx; self.ay_ty=wy
                        self.ay_lock_set=True

                self.drive(self.ay_tx,self.ay_ty,self.alt_t,0,dt)

                dd=math.sqrt((self.x-self.ay_tx)**2+(self.y-self.ay_ty)**2)
                if dd<0.4:
                    self.st=ST_AYRIL_LAND; self._rtz=self.z
                    print(f"[{self.ID}] {'='*48}")
                    print(f"[{self.ID}] {self.ay_renk} TAM MERKEZDE - INIS BASLADI")
                    print(f"[{self.ID}]   Inis GPS hedefi: ({self.ay_tx:.2f}, {self.ay_ty:.2f})")
                    print(f"[{self.ID}]   Drone GPS      : ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})")
                    print(f"[{self.ID}] {'='*48}")

        elif s==ST_AYRIL_LAND:
            if self._rtz<0: self._rtz=self.z
            self._rtz-=0.3*dt
            if self._rtz<LAND_ALT: self._rtz=LAND_ALT

            self.vision_ms += self.ts
            if self.vision_ms >= 300:
                self.vision_ms = 0
                if self.cam is not None:
                    cx_o, cy_o, ok = detect_color(self.cam, self.ay_renk)
                    if ok:
                        wx,wy=self.p2w(cx_o, cy_o)
                        self.ay_tx=self.ay_tx*0.85+wx*0.15
                        self.ay_ty=self.ay_ty*0.85+wy*0.15

            self.drive(self.ay_tx,self.ay_ty,self._rtz,0,dt)
            if self.z<=LAND_ALT+0.05:
                self.moff(); self.pid.reset(); self.sm_tz=None
                self.ay_ms=0; self._rtz=-1; self.st=ST_AYRIL_WAIT
                print(f"[{self.ID}] {self.ay_renk} INDI! Bekleniyor...")

        elif s==ST_AYRIL_WAIT:
            self.moff()
            self.ay_ms+=self.ts
            if self.ay_ms>=self.ay_bek:
                self.pid.reset(); self.sm_tz=None; self.st=ST_AYRIL_RISE
                print(f"[{self.ID}] Bekleme bitti->KALKIS")

        elif s==ST_AYRIL_RISE:
            self.drive(self.x,self.y,self.alt_t,0,dt)
            if self.z>=self.alt_t-0.15:
                self.st=ST_AYRIL_REJ
                print(f"[{self.ID}] Irtifa OK->KATILIM")

        elif s==ST_AYRIL_REJ:
            tx,ty=self.fpos(self.ay_cx,self.ay_cy,self.hdg)
            self.drive(tx,ty,self.alt_t,clamp(self.ye(self.hdg)*2,-0.5,0.5),dt)
            dd=math.sqrt((self.x-tx)**2+(self.y-ty)**2)
            if dd<REACH_XY and abs(self.z-self.alt_t)<REACH_Z:
                self.cx=self.ay_cx; self.cy=self.ay_cy
                self.st=ST_HOVER
                print(f"[{self.ID}] SURUYE KATILDI!")

        # ========== EVE DONUS ==========
        elif s==ST_RA:
            # ST_RA sadece peer sync tetiklemesi için kullanılır.
            # Grup bekleme yok: hemen ST_RXY'ye geç.
            # ID1/ID2 mrx() içinde ps==ST_RA gördüğünde RTL'e katılır.
            self.st=ST_RXY; self.mtx()

        elif s==ST_RXY:
            self.drive(self.hx,self.hy,self.ra_alt,0,dt)
            if math.sqrt((self.x-self.hx)**2+(self.y-self.hy)**2)<REACH_XY:
                self.st=ST_RZ; self.lms=0; self._rtz=-1

        elif s==ST_RZ:
            if self._rtz<0: self._rtz=self.z
            self._rtz-=0.3*dt
            if self._rtz<LAND_ALT: self._rtz=LAND_ALT
            self.drive(self.hx,self.hy,self._rtz,0,dt)
            if self.z<=LAND_ALT:
                self.lms+=self.ts
                if self.lms>=1000:
                    self.moff(); self.pid.reset(); self.sm_tz=None
                    self.lms=0; self._rtz=-1; self.st=ST_IDLE
                    print(f"[{self.ID}] INDI!")
            else: self.lms=0

        elif s==ST_REJ:
            if self.z<self.alt_t-0.15:
                self.drive(self.x,self.y,self.alt_t,0,dt)
            else:
                tx,ty=self.fpos(self.cx,self.cy,self.hdg)
                self.drive(tx,ty,self.alt_t,clamp(self.ye(self.hdg)*2,-0.5,0.5),dt)
                dd=math.sqrt((self.x-tx)**2+(self.y-ty)**2)
                if dd<REACH_XY and abs(self.z-self.alt_t)<REACH_Z:
                    self.st=ST_HOVER

    # ---- Klavye ----
    def _cur_center(self):
        if self.form>=0 and self.form<len(LOC):
            sag=LOC[self.form][self.ID][0]
            ileri=LOC[self.form][self.ID][1]
        else:
            sag=0; ileri=0
        ch=math.cos(self.hdg); sh=math.sin(self.hdg)
        return self.x - ileri*ch - sag*sh, self.y - ileri*sh + sag*ch

    def keys(self):
        k=self.kb.getKey()
        if k<=0:
            self._last_arrow=-1   # tuş bırakıldı, debounce sıfırla
            return
        c=chr(k) if 0<k<256 else ""
        flying=self.st not in (ST_IDLE,ST_TAKEOFF,ST_RZ,ST_RA,ST_RXY)

        # ===== OK TUŞLARI: yarı-otonom hareket (ID0, HOVER) =====
        if self.manual and self.ID==0 and self.st==ST_HOVER:
            if k in (Keyboard.UP, Keyboard.DOWN, Keyboard.LEFT, Keyboard.RIGHT):
                if k!=self._last_arrow:
                    self._last_arrow=k
                    if k==Keyboard.UP:
                        self.cx+=math.cos(self.hdg)*self.man_spd
                        self.cy+=math.sin(self.hdg)*self.man_spd
                    elif k==Keyboard.DOWN:
                        self.cx-=math.cos(self.hdg)*self.man_spd
                        self.cy-=math.sin(self.hdg)*self.man_spd
                    elif k==Keyboard.LEFT:
                        self.cx+=math.cos(self.hdg+math.pi/2)*self.man_spd
                        self.cy+=math.sin(self.hdg+math.pi/2)*self.man_spd
                    elif k==Keyboard.RIGHT:
                        self.cx+=math.cos(self.hdg-math.pi/2)*self.man_spd
                        self.cy+=math.sin(self.hdg-math.pi/2)*self.man_spd
                    self.mtx()
                return  # ok tuşu işlendi

        if c and c in "Tt" and self.st==ST_IDLE:
            self.pid.reset(); self.sm_tz=None; fly=False
            for i in range(NUM_UAVS):
                if i==self.ID or self.na[i]>2000: continue
                ns=self.nb[i].get("st",ST_IDLE) if self.nb[i] else ST_IDLE
                if ns in (ST_HOVER,ST_GOTO,ST_WAIT,ST_MANV,ST_ALIGN,ST_AG,ST_QR_SCAN):
                    self.form=self.nb[i]["form"]; self.hdg=self.nb[i]["hdg"]
                    self.cx=self.nb[i]["cx"]; self.cy=self.nb[i]["cy"]
                    self.qr=self.nb[i]["qr"]; fly=True; break
            if fly:
                self.manual=True; self.st=ST_REJ
                print(f"[{self.ID}] Suruye katiliyor [YARI-OTONOM]"); self.mtx()
            else:
                self.manual=True; self.form=FLINE; self.hdg=0
                self.cx,self.cy=self._cur_center()
                self.st=ST_TAKEOFF
                print(f"[{self.ID}] KALKIS! [YARI-OTONOM]"); self.mtx()

        # ===== 7/8/9: BIREYSEL RTL =====
        # FIX-15: manual bayragi korunuyor, sadece kendi RTL'i tetikleniyor
        elif c=="7" and self.ID==0 and flying:
            self.ra_alt=self.z; self.lms=0; self._rtz=-1
            # manual=False YAPILMIYOR: sürü sync bozulmasın
            self.st=ST_RXY; print(f"[0] Bireysel RTL -> eve")
        elif c=="8" and self.ID==1 and flying:
            self.ra_alt=self.z; self.lms=0; self._rtz=-1
            self.st=ST_RXY; print(f"[1] Bireysel RTL -> eve")
        elif c=="9" and self.ID==2 and flying:
            self.ra_alt=self.z; self.lms=0; self._rtz=-1
            self.st=ST_RXY; print(f"[2] Bireysel RTL -> eve")

        elif self.ID==0 and flying:

            # 0: Otonom QR moduna geç
            if c=="0" and self.manual:
                self.manual=False
                self.qr=0
                # FIX-15: cx/cy'yi QR1 hedefine ayarla, yoksa fpos yanlış slot hesaplıyor
                # GOTO ve QRSCAN'da cx/cy = hedef QR koordinatı olmalı
                # Ama önce ALIGN lazım; cx/cy şimdilik mevcut merkez kalıyor,
                # hdg QR1'e bakacak, ALIGN->GOTO->QRSCAN zinciri cx/cy'yi QR1'e set edecek
                dx=QRX[0]-self.cx; dy=QRY[0]-self.cy
                if math.sqrt(dx*dx+dy*dy)>0.5: self.hdg=math.atan2(dy,dx)
                self.atx,self.aty=self.fpos(self.cx,self.cy,self.hdg)
                self.ams=0
                for i in range(NUM_UAVS): self.adone[i]=0
                self.st=ST_ALIGN
                print(f"[OTONOM] *** QR MODU - QR1 hedefleniyor (cx={self.cx:.1f},cy={self.cy:.1f}) ***")
                self.mtx()

            elif c and c in "Rr":
                self.srtla()

            elif self.manual:
                if c and c in "Qq":
                    self.alt_t=min(self.alt_t+0.2,MAX_ALT)
                    print(f"[PILOT] Irtifa: {self.alt_t:.1f}m"); self.mtx()
                elif c and c in "Ee":
                    self.alt_t=max(self.alt_t-0.2,0.5)
                    print(f"[PILOT] Irtifa: {self.alt_t:.1f}m"); self.mtx()
                elif c and c in "Vv":
                    self.form=FV; self.cx,self.cy=self._cur_center()
                    print(f"[PILOT] Formasyon: V"); self.mtx()
                elif c and c in "Ll":
                    self.form=FLINE; self.cx,self.cy=self._cur_center()
                    print(f"[PILOT] Formasyon: CIZGI"); self.mtx()
                elif c and c in "Aa":
                    self.form=FARR; self.cx,self.cy=self._cur_center()
                    print(f"[PILOT] Formasyon: OKBASI"); self.mtx()

            elif not self.manual:
                if c and c in "123456" and self.st in (ST_HOVER,ST_WAIT,ST_MANV,
                        ST_GOTO,ST_ALIGN,ST_AG,ST_QR_SCAN):
                    qi=int(c)-1; self.qr=qi
                    dx=QRX[qi]-self.cx; dy=QRY[qi]-self.cy
                    if math.sqrt(dx*dx+dy*dy)>0.5: self.hdg=math.atan2(dy,dx)
                    self.atx,self.aty=self.fpos(self.cx,self.cy,self.hdg)
                    self.ams=0
                    for i in range(NUM_UAVS): self.adone[i]=0
                    self.st=ST_ALIGN; self.mtx()
                elif c and c in "Vv": self.form=FV; self.mtx()
                elif c and c in "Ll": self.form=FLINE; self.mtx()
                elif c and c in "Aa": self.form=FARR; self.mtx()

    def sens(self,dt):
        if self.imu:
            r=self.imu.getRollPitchYaw()
            self.roll=r[0]; self.pitch=r[1]; self.yaw=r[2]
        if self.gyro: self.yr=self.gyro.getValues()[2]
        if self.gps and dt>1e-6:
            v=self.gps.getValues(); xg=v[0]; yg=v[1]; self.z=v[2]
            if not self.sset:
                self.sx=xg; self.sy=yg; self.hx=xg; self.hy=yg
                self.sset=True; print(f"[{self.ID}] Start:({self.sx:.2f},{self.sy:.2f})")
            c=math.cos(self.yaw); s=math.sin(self.yaw)
            vxg=(xg-self.px)/dt; vyg=(yg-self.py)/dt
            self.vx=vxg*c+vyg*s; self.vy=-vxg*s+vyg*c
            self.x=xg; self.y=yg; self.px=xg; self.py=yg

    def run(self):
        while self.robot.step(self.ts)!=-1:
            if self.robot.getTime()>2: break
        print(f"[{self.ID}] Hazir. T'ye bas!")
        self.pt=self.robot.getTime()
        while self.robot.step(self.ts)!=-1:
            now=self.robot.getTime(); dt=now-self.pt
            self.sens(dt); self.mrx(); self.keys(); self.step_ctrl(dt)
            self._ba+=self.ts
            if self._ba>=BCAST_MS: self.mtx(); self._ba=0
            self._la+=self.ts
            lp=400 if self.st in (ST_TAKEOFF,ST_AA,ST_AG,ST_ALIGN,ST_GOTO,
                ST_MANV,ST_REJ,ST_RA,ST_QR_SCAN,ST_AYRIL_GOTO,ST_AYRIL_LAND) else 2000
            if self._la>=lp:
                self._la=0
                sn=STNAMES[self.st] if 0<=self.st<len(STNAMES) else "?"
                mn=" [PILOT]" if self.manual else ""
                print(f"[{self.ID}] ({self.x:.1f},{self.y:.1f},{self.z:.2f}) "
                      f"alt={self.alt_t:.1f} st={sn} f={self.form} "
                      f"qr={self.qr+1} hdg={math.degrees(self.hdg):.0f}{mn}")
            self.pt=now
        self.robot.cleanup()
