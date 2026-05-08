"""
qr_gorev.py - QR Gorev sinifi ve parse fonksiyonu
"""

import json
from constants import FNONE, FV, FLINE, FARR, TAKEOFF_ALT, TEAM_ID
from utils import map_form

# ====================== QR GOREV ======================
class QRGorev:
    def __init__(self):
        self.valid=False; self.qr_id=-1
        self.form_aktif=False; self.form_tip=FNONE
        self.manv_aktif=False; self.pitch_deg=0.0; self.roll_deg=0.0
        self.irtifa_aktif=False; self.irtifa_m=TAKEOFF_ALT
        self.bekle_ms=3000
        self.ayril_aktif=False; self.ayril_drone_id=-1
        self.ayril_renk=""; self.ayril_bekle_ms=0
        self.sonraki=[-1,-1,-1]

def parse_qr(js):
    out=QRGorev()
    try: data=json.loads(js)
    except: print("[QR] JSON hata!"); return None

    if "q" in data and "gorev" not in data:
        out.qr_id=int(data.get("q",-1))
        if out.qr_id<0: return None
        f=data.get("f","")
        out.form_aktif=bool(f)
        out.form_tip=map_form(f)
        p=float(data.get("p",0)); r=float(data.get("r",0))
        out.manv_aktif=(p!=0 or r!=0)
        out.pitch_deg=p; out.roll_deg=r
        iv=float(data.get("i",0))
        out.irtifa_aktif=(iv>0)
        out.irtifa_m=iv if iv>0 else TAKEOFF_ALT
        out.bekle_ms=int(data.get("b",3))*1000
        out.ayril_aktif=bool(data.get("sa",0))
        out.ayril_drone_id=int(data.get("sd",-1))
        out.ayril_renk=str(data.get("sr",""))
        sb=data.get("sb",5)
        out.ayril_bekle_ms=int(sb)*1000 if sb else 5000
        for i,k in enumerate(["n1","n2","n3"]):
            v=int(data.get(k,0))
            out.sonraki[i]=v-1 if v>0 else -1
    else:
        out.qr_id=data.get("qr_id",-1)
        if out.qr_id<0: return None
        g=data.get("gorev",{})
        fo=g.get("formasyon",{})
        out.form_aktif=bool(fo.get("aktif",False))
        out.form_tip=map_form(fo.get("tip",""))
        mo=g.get("manevra_pitch_roll",{})
        out.manv_aktif=bool(mo.get("aktif",False))
        out.pitch_deg=float(mo.get("pitch_deg",0))
        out.roll_deg=float(mo.get("roll_deg",0))
        io=g.get("irtifa_degisim",{})
        out.irtifa_aktif=bool(io.get("aktif",False))
        out.irtifa_m=float(io.get("deger",TAKEOFF_ALT))
        out.bekle_ms=int(g.get("bekleme_suresi_s",3))*1000
        ao=g.get("suruden_ayrilma",{})
        out.ayril_aktif=bool(ao.get("aktif",False))
        aid=ao.get("ayrilacak_drone_id",-1)
        out.ayril_drone_id=int(aid) if aid is not None else -1
        out.ayril_renk=str(ao.get("hedef_renk","") or "")
        bk=ao.get("bekleme_suresi_s",0)
        out.ayril_bekle_ms=int(bk)*1000 if bk else 5000
        sq=data.get("sonraki_qr",{})
        for i,k in enumerate(["team_1","team_2","team_3"]):
            v=int(sq.get(k,0) or 0)
            out.sonraki[i]=v-1 if v>0 else -1

    out.valid=True
    FNAMES={FNONE:"YOK",FV:"V_DIZILIS",FLINE:"CIZGI",FARR:"OKBASI"}
    print(f"\n{'='*50}")
    print(f"  QR{out.qr_id} GOREVI COZULDU")
    print(f"{'='*50}")
    print(f"  Formasyon : {FNAMES.get(out.form_tip,'?')} {'(aktif)' if out.form_aktif else '(pasif)'}")
    print(f"  Irtifa    : {out.irtifa_m:.1f} m {'(aktif)' if out.irtifa_aktif else '(pasif)'}")
    if out.manv_aktif:
        print(f"  Manevra   : pitch={out.pitch_deg}° roll={out.roll_deg}° (aktif)")
    else:
        print(f"  Manevra   : YOK")
    if out.ayril_aktif:
        print(f"  Ayrilma   : Drone{out.ayril_drone_id} -> {out.ayril_renk} ({out.ayril_bekle_ms/1000:.0f}s)")
    else:
        print(f"  Ayrilma   : YOK")
    print(f"  Bekleme   : {out.bekle_ms/1000:.0f} s")
    nxt_str=[]
    for i,tn in enumerate(["T1","T2","T3"]):
        n=out.sonraki[i]
        nxt_str.append(f"{tn}->QR{n+1}" if n>=0 else f"{tn}->BITIS")
    print(f"  Sonraki   : {', '.join(nxt_str)}")
    print(f"{'='*50}\n")
    return out
