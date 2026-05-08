"""
utils.py - Yardimci fonksiyonlar
"""

from constants import FNONE, FV, FLINE, FARR

# ====================== YARDIMCI ======================
def clamp(v,lo,hi):
    return lo if v<lo else (hi if v>hi else v)

def map_form(tip):
    if not tip: return FNONE
    t=tip.upper().strip()
    if t in ("OK","OKBASI"): return FARR
    if t in ("V","V_DIZILIS","V_D"): return FV
    if t in ("CZ","CIZGI","LINE"): return FLINE
    return FNONE
