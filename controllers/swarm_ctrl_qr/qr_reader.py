"""
qr_reader.py - QR kod okuma fonksiyonu
"""

import numpy as np
from constants import OPENCV_OK

if OPENCV_OK:
    import cv2
    _QR_DET = cv2.QRCodeDetector()
else:
    cv2 = None
    _QR_DET = None

# ====================== QR OKUMA ======================
def try_decode_qr(cam):
    if not OPENCV_OK or cam is None or _QR_DET is None: return None
    raw=cam.getImage()
    if raw is None: return None
    w=cam.getWidth(); h=cam.getHeight()
    if w<=0 or h<=0: return None
    img=np.frombuffer(raw,dtype=np.uint8).reshape((h,w,4))
    y1,y2 = h//4, h*3//4
    x1,x2 = w//4, w*3//4
    crop = img[y1:y2, x1:x2, :3]
    t,_,_=_QR_DET.detectAndDecode(crop)
    if t: print(f"[QR] Y1 OK"); return t
    gray=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
    cw,ch = x2-x1, y2-y1
    big=cv2.resize(gray,(cw*2,ch*2),interpolation=cv2.INTER_LINEAR)
    _,otsu=cv2.threshold(big,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    t,_,_=_QR_DET.detectAndDecode(cv2.cvtColor(otsu,cv2.COLOR_GRAY2BGR))
    if t: print(f"[QR] Y2(otsu) OK"); return t
    return None
