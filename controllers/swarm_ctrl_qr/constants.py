"""
constants.py - Tum sabitler
"""

import math
import numpy as np

try:
    import cv2
    OPENCV_OK = True
except ImportError:
    OPENCV_OK = False
    print("[UYARI] OpenCV bulunamadi!")

# ====================== SABITLER ======================
NUM_UAVS    = 3
TEAM_ID     = 0          # 0=team_1
MAX_QR      = 8
TAKEOFF_ALT = 15
REACH_XY    = 0.3
REACH_Z     = 0.25
LAND_ALT    = 0.10
ALIGN_TOL   = 0.10
ALIGN_HOLD  = 1500
OFFSET      = 1.5
QR_WAIT_MS  = 3000
BCAST_MS    = 50
COL_R       = 0.8
QR_SCAN_TIMEOUT_MS  = 20000
QR_SCAN_ALT         = 15.0
MAX_ALT             = 50.0
CAM_FOV     = 1.309

BASE_THRUST = 48.0
ALT_KP = 10.0; ALT_KI = 5.0; ALT_KD = 5.0

MAX_Z_RATE  = 1.0

EWMA_ALPHA  = 0.3
COLOR_MIN_PX = 200

QRX = [10.0, 15.0, 25.0, 30.0, 25.0, 15.0]
QRY = [ 5.0, 15.0, 15.0,  5.0, -5.0, -5.0]
HX  = [0.0, 0.0, 0.0]
HY  = [0.0, 1.5, -1.5]

# ====================== FORMASYON ======================
FNONE = -1; FV = 0; FLINE = 1; FARR = 2
LOC = [
    [[-OFFSET,+OFFSET],[0,0],[+OFFSET,+OFFSET]],   # V
    [[-OFFSET,0],      [0,0],[+OFFSET,0]],          # CIZGI
    [[-OFFSET,-OFFSET],[0,0],[+OFFSET,-OFFSET]],    # OKBASI
]

# ====================== DURUMLAR ======================
ST_IDLE=0; ST_TAKEOFF=1; ST_AA=2; ST_AG=3; ST_QR_SCAN=4
ST_HOVER=5; ST_ALIGN=6; ST_GOTO=7; ST_WAIT=8; ST_MANV=9
ST_RA=10; ST_RXY=11; ST_RZ=12; ST_REJ=13
ST_AYRIL_GOTO=14; ST_AYRIL_LAND=15; ST_AYRIL_WAIT=16
ST_AYRIL_RISE=17; ST_AYRIL_REJ=18

STNAMES = ["IDLE","TKOFF","AA","AG","QRSCAN","HOVER","ALIGN","GOTO",
           "WAIT","MANV","RA","RXY","RZ","REJ",
           "AY_GOTO","AY_LAND","AY_WAIT","AY_RISE","AY_REJ"]

PKT_STATE=0; PKT_QR=1

BLUE_LO  = np.array([90,80,50]);  BLUE_HI = np.array([130,255,255])
RED_LO1  = np.array([0,80,50]);   RED_HI1 = np.array([12,255,255])
RED_LO2  = np.array([160,80,50]); RED_HI2 = np.array([180,255,255])
