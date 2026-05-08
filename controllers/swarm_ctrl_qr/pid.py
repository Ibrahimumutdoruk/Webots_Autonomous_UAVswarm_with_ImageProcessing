"""
pid.py - PID kontrolcu sinifi
"""

from constants import ALT_KP, ALT_KI, ALT_KD, BASE_THRUST
from utils import clamp

# ====================== PID ======================
class PID:
    def __init__(self): self.reset()
    def reset(self):
        self._pA=0; self._pP=0; self._pR=0
        self._pVx=0; self._pVy=0; self._aI=0
    def alt(self,des,act,dt):
        e=des-act; de=(e-self._pA)/dt
        self._aI=clamp(self._aI+e*dt,-2,2); self._pA=e
        return ALT_KP*clamp(e,-1,1)+ALT_KI*self._aI+ALT_KD*de+BASE_THRUST
    def vel(self,vxd,vyd,vxa,vya,dt):
        ex=vxd-vxa; ey=vyd-vya
        dex=(ex-self._pVx)/dt; dey=(ey-self._pVy)/dt
        self._pVx=ex; self._pVy=ey
        return 2*clamp(ex,-1,1)+0.5*dex, -2*clamp(ey,-1,1)-0.5*dey
    def att(self,rd,pd,ra,pa,yrd,yra,dt):
        re=rd-ra; rde=(re-self._pR)/dt
        pe=pd-pa; pde=(pe-self._pP)/dt
        self._pR=re; self._pP=pe
        return (0.5*clamp(re,-1,1)+0.1*rde,
                -0.5*clamp(pe,-1,1)-0.1*pde,
                clamp(yrd-yra,-1,1))
    @staticmethod
    def mix(a,r,p,y):
        return (clamp(a-r+p+y,0,600), clamp(a-r-p-y,0,600),
                clamp(a+r-p+y,0,600), clamp(a+r+p-y,0,600))
