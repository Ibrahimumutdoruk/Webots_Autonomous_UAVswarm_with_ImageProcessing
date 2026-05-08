"""
color_detect.py - Renk tespiti fonksiyonu
"""

import numpy as np
from constants import OPENCV_OK, BLUE_LO, BLUE_HI, RED_LO1, RED_HI1, RED_LO2, RED_HI2, COLOR_MIN_PX

if OPENCV_OK:
    import cv2
else:
    cv2 = None

# ====================== RENK TESPITI ======================
def detect_color(cam, renk):
    if not OPENCV_OK or cam is None: return 0,0,False
    w=cam.getWidth(); h=cam.getHeight()
    raw=cam.getImage()
    if raw is None: return 0,0,False
    img=np.frombuffer(raw,dtype=np.uint8).reshape((h,w,4))
    cy1=h//4
    bgr=img[cy1:, :, :3]
    ch=h-cy1
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)

    if renk.upper()=="MAVI":
        mask=cv2.inRange(hsv,BLUE_LO,BLUE_HI)
    elif renk.upper()=="KIRMIZI":
        mask=cv2.bitwise_or(cv2.inRange(hsv,RED_LO1,RED_HI1),
                            cv2.inRange(hsv,RED_LO2,RED_HI2))
    else: return 0,0,False

    if cv2.countNonZero(mask)<COLOR_MIN_PX:
        return 0,0,False

    M=cv2.moments(mask)
    if M["m00"]==0: return 0,0,False
    px=M["m10"]/M["m00"]
    py=M["m01"]/M["m00"] + cy1
    return (px-w/2)/(w/2), (py-h/2)/(h/2), True
