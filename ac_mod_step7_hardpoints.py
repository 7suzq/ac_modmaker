# -*- coding: utf-8 -*-
"""
操作7: サス役割の幾何分類 → DWBハードポイント抽出 → AC物理座標 → ini テキスト出力＋ファイル書き出し
================================================================================
前提: ac_mod_build.py 実行済み（WHEEL_XX 必要）。.blend は保存済みであること（同フォルダに書き出すため）。

・フロント(A字あり): アッパー=A字、ロア=アウター最近接の単リンク2脚、タイロッド=残り、プッシュ=最急傾斜(除外)。
・リア(A字なし): 高さで 2本=アッパー脚 / 2本=ロア脚 / 中央1本=トーリンク（※幾何のみだと曖昧→暫定。ROLE_OVERRIDEで確定可）。
・前/後脚は車体側Y（-Y=前）で振り分け。

出力: ホイール中心原点、X=インボード/Y=上/Z=前、メートル(×scale_length)。
  FRONT は LF、REAR は LR を基準に算出（ACは反対側を自動ミラー）。
  → レポートにコピペ用iniテキスト＋ .blend と同じフォルダに suspensions_geometry.ini を書き出し。

★このモデルの正解iniは無いので値は「ジオメトリ由来の出発点」。最終は in-game 検証。
実行: Scripting で Run。
"""

import bpy, os
import numpy as np
from mathutils import Vector, Matrix
from math import radians, degrees

# ============================== CONFIG ==============================
ARM_NUMS = [None, 2, 3, 4, 5, 6, 7]
APEX_ANGLE_MIN = 30.0
INBOARD_QUANTILE = 0.6
PLACE_EMPTIES = True
HP_SIZE = 0.02
OUT_NAME = "suspensions_geometry.ini"

# 役割手動上書き(自動分類より優先)。例(リア確定時など):
# ROLE_OVERRIDE = {"bl": {"upper":["suspension2_bl","suspension3_bl"],
#                         "lower":["suspension_bl","suspension4_bl"],
#                         "steer":"suspension5_bl"}}
ROLE_OVERRIDE = {}

CORNER_SUFFIX = {"LF": "fl", "RF": "fr", "LR": "bl", "RR": "br"}
AXLE_REF = {"FRONT": "LF", "REAR": "LR"}     # iniの基準コーナー(左側)
AC_BASE = Matrix.Rotation(radians(90), 4, 'X')
ORDER = ["WBCAR_TOP_FRONT", "WBCAR_TOP_REAR", "WBCAR_BOTTOM_FRONT", "WBCAR_BOTTOM_REAR",
         "WBTYRE_TOP", "WBTYRE_BOTTOM", "WBCAR_STEER", "WBTYRE_STEER"]
# ===================================================================

REPORT = []
def log(s): REPORT.append(s); print(s)
def fmtv(v): return "(%.4f,%.4f,%.4f)" % (v.x, v.y, v.z)

def scale_length():
    return bpy.context.scene.unit_settings.scale_length or 1.0

def world_verts(obj):
    dg = bpy.context.evaluated_depsgraph_get()
    ev = obj.evaluated_get(dg)
    try: me = ev.to_mesh()
    except Exception: return []
    mw = obj.matrix_world
    pts = [mw @ v.co for v in me.vertices]
    ev.to_mesh_clear()
    return pts

def np_pts(obj):
    p = world_verts(obj)
    return np.array([[v.x, v.y, v.z] for v in p]) if p else None

def cluster_two(A):
    c = A.mean(0)
    i0 = A[int(np.argmax(np.linalg.norm(A - c, axis=1)))]
    i1 = A[int(np.argmax(np.linalg.norm(A - i0, axis=1)))]
    d0 = np.linalg.norm(A - i0, axis=1); d1 = np.linalg.norm(A - i1, axis=1)
    g0 = A[d0 <= d1]; g1 = A[d1 < d0]
    if len(g0) == 0 or len(g1) == 0: return A.mean(0), A.mean(0)
    return g0.mean(0), g1.mean(0)

def single_chassis(P, wc):
    d = np.linalg.norm(P - np.array([wc.x, wc.y, wc.z]), axis=1)
    far = P[int(d.argmax())]
    L = float(np.linalg.norm(P.max(0) - P.min(0)))
    near = P[np.linalg.norm(P - far, axis=1) <= 0.20 * L]
    return Vector(near.mean(0))

def analyze(obj, wc):
    P = np_pts(obj)
    if P is None or len(P) < 3: return None
    wcn = np.array([wc.x, wc.y, wc.z])
    d = np.linalg.norm(P - wcn, axis=1)
    tyre = Vector(P[int(d.argmin())])
    L = float(np.linalg.norm(P.max(0) - P.min(0)))
    inb = P[d >= np.quantile(d, INBOARD_QUANTILE)]
    if len(inb) < 2: inb = P[d >= np.median(d)]
    c0, c1 = cluster_two(inb)
    j0, j1 = Vector(c0), Vector(c1)
    v0 = j0 - tyre; v1 = j1 - tyre
    apex = degrees(v0.angle(v1)) if (v0.length > 1e-6 and v1.length > 1e-6) else 0.0
    is_a = apex >= APEX_ANGLE_MIN
    chassis = (j0 + j1) * 0.5 if is_a else single_chassis(P, wc)
    incline = abs(tyre.z - chassis.z) / max(L, 1e-6)
    return dict(name=obj.name, tyre=tyre, j0=j0, j1=j1, apex=apex, L=L,
                is_a=is_a, chassis=chassis, incline=incline)

def arm_data(suffix, wc):
    out = []
    for n in ARM_NUMS:
        nm = "suspension_%s" % suffix if n is None else "suspension%d_%s" % (n, suffix)
        o = bpy.data.objects.get(nm)
        if o:
            a = analyze(o, wc)
            if a: out.append(a)
    return out

def by_name(data, nm):
    for a in data:
        if a['name'] == nm: return a
    return None

def closest_pair(items):
    best = None; bd = 1e18
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            dd = (items[i]['tyre'] - items[j]['tyre']).length
            if dd < bd: bd = dd; best = (items[i], items[j])
    return best

def classify(data, suffix):
    if suffix in ROLE_OVERRIDE:
        ov = dict(ROLE_OVERRIDE[suffix]); ov['src'] = "override"; return ov
    aarms = sorted([a for a in data if a['is_a']], key=lambda a: a['chassis'].z)
    singles = [a for a in data if not a['is_a']]
    roles = dict(upper=None, lower=None, steer=None, push=None, src="auto")
    if aarms:   # ---- フロント型 ----
        roles['upper'] = aarms[-1]['name']
        lower_aarm = aarms[0]['name'] if len(aarms) >= 2 else None
        if singles:
            push = max(singles, key=lambda a: a['incline']); roles['push'] = push['name']
            rest = [a for a in singles if a['name'] != push['name']]
        else:
            rest = []
        if lower_aarm:
            roles['lower'] = lower_aarm
        elif len(rest) >= 2:
            p = closest_pair(rest)
            roles['lower'] = [p[0]['name'], p[1]['name']]
            rest = [a for a in rest if a['name'] not in roles['lower']]
        if rest: roles['steer'] = rest[0]['name']
    else:       # ---- リア型(A字なし): 高さで2+2+1 ----
        s = sorted(singles, key=lambda a: (a['chassis'].z + a['tyre'].z) * 0.5)
        roles['src'] = "auto(rear/暫定)"
        if len(s) >= 4:
            roles['lower'] = [s[0]['name'], s[1]['name']]
            roles['upper'] = [s[-1]['name'], s[-2]['name']]
            mid = [a for a in s if a['name'] not in roles['lower'] + roles['upper']]
            if mid: roles['steer'] = mid[0]['name']
    return roles

def front_rear_joints(a):
    j = sorted([a['j0'], a['j1']], key=lambda v: v.y)   # 前=-Y
    return j[0], j[1]

def wishbone_points(data, spec):
    """戻り: (car_front, car_rear, tyre) world。"""
    if spec is None: return None
    if isinstance(spec, str):                  # A字
        a = by_name(data, spec)
        if a is None: return None
        f, r = front_rear_joints(a)
        return f, r, a['tyre']
    a0 = by_name(data, spec[0]); a1 = by_name(data, spec[1])   # 2脚
    if not a0 or not a1: return None
    legs = sorted([a0, a1], key=lambda a: a['chassis'].y)      # 前=-Y
    return legs[0]['chassis'], legs[1]['chassis'], (legs[0]['tyre'] + legs[1]['tyre']) * 0.5

def hardpoints(data, roles):
    hp = {}
    up = wishbone_points(data, roles.get('upper'))
    if up: hp['WBCAR_TOP_FRONT'], hp['WBCAR_TOP_REAR'], hp['WBTYRE_TOP'] = up
    lo = wishbone_points(data, roles.get('lower'))
    if lo: hp['WBCAR_BOTTOM_FRONT'], hp['WBCAR_BOTTOM_REAR'], hp['WBTYRE_BOTTOM'] = lo
    st = roles.get('steer')
    if st:
        a = by_name(data, st)
        if a: hp['WBCAR_STEER'] = a['chassis']; hp['WBTYRE_STEER'] = a['tyre']
    return hp

def to_ac(p, wc, sl):
    dp = p - wc
    sx = -1.0 if wc.x >= 0 else 1.0
    return (sx * dp.x * sl, dp.z * sl, -dp.y * sl)

def car_collection():
    for nm in ["x0_tyre_fl", "wheel_fl", "x0_tyre_bl"]:
        o = bpy.data.objects.get(nm)
        if o and o.users_collection: return o.users_collection[0]
    return bpy.context.scene.collection

def place_hp(name, p, coll):
    e = bpy.data.objects.get(name)
    if e is None:
        e = bpy.data.objects.new(name, None); coll.objects.link(e)
    for c in list(e.users_collection): c.objects.unlink(e)
    coll.objects.link(e)
    e.empty_display_type = 'PLAIN_AXES'; e.empty_display_size = HP_SIZE
    e.parent = None
    e.matrix_world = Matrix.Translation(p) @ AC_BASE
    return e


def main():
    sl = scale_length()
    log("============ 操作7: DWBハードポイント抽出 → ini ============")
    log("scale_length=%.4f (m/BU)  座標系: 原点=ホイール中心, X=インボード, Y=上, Z=前" % sl)
    coll = car_collection()
    ini_lines = ["; === suspensions geometry extracted from Blender (operation 7) ==="]

    # TRACK / WHEELBASE (参考)
    wlf = bpy.data.objects.get("WHEEL_LF"); wrf = bpy.data.objects.get("WHEEL_RF")
    wlr = bpy.data.objects.get("WHEEL_LR")
    if wlf and wrf:
        tf = abs(wlf.matrix_world.translation.x - wrf.matrix_world.translation.x) * sl
        ini_lines.append("; TRACK_FRONT=%.4f" % tf)
    if wlf and wlr:
        wb = abs(wlf.matrix_world.translation.y - wlr.matrix_world.translation.y) * sl
        ini_lines.append("; WHEELBASE=%.4f" % wb)

    for axle in ["FRONT", "REAR"]:
        corner = AXLE_REF[axle]; suffix = CORNER_SUFFIX[corner]
        wheel = bpy.data.objects.get("WHEEL_%s" % corner)
        if not wheel:
            log("  [%s] WHEEL_%s 無し→スキップ" % (axle, corner)); continue
        wc = wheel.matrix_world.translation.copy()
        data = arm_data(suffix, wc)
        roles = classify(data, suffix)
        log("  [%s/%s] 分類(%s): upper=%s lower=%s steer=%s push=%s"
            % (axle, corner, roles['src'], roles['upper'], roles['lower'],
               roles['steer'], roles['push']))
        for a in data:
            log("      %-16s %s apex=%5.1f° H=%.3f incline=%.2f tyre=%s"
                % (a['name'], "A字" if a['is_a'] else "単 ", a['apex'],
                   (a['chassis'].z + a['tyre'].z) * 0.5, a['incline'], fmtv(a['tyre'])))

        hp = hardpoints(data, roles)
        ini_lines.append("")
        ini_lines.append("[%s]" % axle)
        ini_lines.append("TYPE=DWB")
        for k in ORDER:
            if k in hp:
                ac = to_ac(hp[k], wc, sl)
                ini_lines.append("%s=%.4f, %.4f, %.4f" % (k, ac[0], ac[1], ac[2]))
                if PLACE_EMPTIES: place_hp(k + "_" + suffix, hp[k], coll)
            else:
                ini_lines.append("; %s=（役割未割当）" % k)
    bpy.context.view_layer.update()

    text = "\n".join(ini_lines)
    # ファイル書き出し(.blend と同じフォルダ)
    blend = bpy.data.filepath
    if blend:
        out = os.path.join(os.path.dirname(blend), OUT_NAME)
        try:
            with open(out, "w", encoding="utf-8") as f: f.write(text + "\n")
            log("書き出し成功: %s" % out)
        except Exception as e:
            log("[WARN] 書き出し失敗: %s" % e)
    else:
        log("[WARN] .blend未保存のためファイル書き出し不可。保存後に再実行してください。")

    log("============================================================")
    print("\n\n##### COPY (レポート) #####")
    print("\n".join(REPORT))
    print("\n##### ↓ suspensions.ini 用テキスト(コピペ可) ↓ #####")
    print(text)
    print("##### ここまで #####")

if __name__ == "__main__":
    main()
