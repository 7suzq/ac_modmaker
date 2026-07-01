# -*- coding: utf-8 -*-
"""
操作6: サス DIR_ ＋ ハードポイント（形状ベース・開き角でA字判定）
================================================================================
前提: ac_mod_build.py 実行済み（WHEEL_XX / HUB_XX 必要）。

各 suspensionN_XX について（番号非依存・純ジオメトリ）:
  ① アウター(tyre) = ホイール中心に最も近い頂点。
  ② 内側(シャーシ側)領域を2クラスタに分け、候補接合 j0/j1 を得る。
  ③ ★A字判定 = tyre を頂点とする j0–j1 の「開き角」。
       開き角 >= APEX_ANGLE_MIN なら A字(ウィッシュボーン)、未満なら単リンク。
       (長い単リンクが間隔比で誤検出される問題を回避)
  ④ A字: cjF/cjR(前=-Y側) を実接合に置く（将来 WBCAR_*_FRONT/REAR）。pivot=中点。
     単リンク: 内側centroid を car 接合に（将来 WBCAR_STEER 等）。
     tyre = DIR_ ターゲット位置（将来 WBTYRE_*）。
  ⑤ DIR_: arm + tether + decal を pointer の子、DIR_p_<arm> を HUB の子。
USE_SAFETYCELL=True なら、各接合が safetycell にどれだけ食い込むか(距離)をレポート(健全性確認)。

まず PROCESS_CORNERS=["LF"]。レポートの apex_deg を見て、本物のA字だけ A になっているか確認。
"""

import bpy
import numpy as np
from mathutils import Vector, Matrix
from math import radians, degrees

# ============================== CONFIG ==============================
PROCESS_CORNERS = ["LF", "RF", "LR", "RR"]
ARM_NUMS = [None, 2, 3, 4, 5, 6, 7]
POINTER_PREFIX = "p_"
DIR_PARENT_TO_HUB = True
INCLUDE_DECALS = True
APEX_ANGLE_MIN = 30.0          # ★この開き角(度)以上でA字。52°=本物, 8〜15°=単リンク。
INBOARD_QUANTILE = 0.6         # 内側(シャーシ側)とみなす距離分位
SIZE_FACTOR = 0.5
RESET_FIRST = True
USE_SAFETYCELL = True          # 接合の safetycell 食い込みを距離でレポート(位置は上書きしない)
SAFETYCELL_NAME = "safetycell"

CORNER_SUFFIX = {"LF": "fl", "RF": "fr", "LR": "bl", "RR": "br"}
AC_BASE = Matrix.Rotation(radians(90), 4, 'X')
# ===================================================================

REPORT = []
def log(s): REPORT.append(s); print(s)
def fmt(v): return "(%.4f,%.4f,%.4f)" % (v.x, v.y, v.z)

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
    if len(g0) == 0 or len(g1) == 0:
        return A.mean(0), A.mean(0)
    return g0.mean(0), g1.mean(0)

def analyze_arm(obj, wc):
    """戻り: tyre, j0, j1, apex_deg, arm_len, is_aarm"""
    P = np_pts(obj)
    if P is None or len(P) < 3:
        return None
    wcn = np.array([wc.x, wc.y, wc.z])
    d = np.linalg.norm(P - wcn, axis=1)
    tyre = Vector(P[int(d.argmin())])                       # アウター(ホイール最近傍)
    arm_len = float(np.linalg.norm(P.max(0) - P.min(0)))
    inb = P[d >= np.quantile(d, INBOARD_QUANTILE)]          # 内側領域
    if len(inb) < 2: inb = P[d >= np.median(d)]
    c0, c1 = cluster_two(inb)
    j0 = Vector(c0); j1 = Vector(c1)
    v0 = j0 - tyre; v1 = j1 - tyre
    apex = degrees(v0.angle(v1)) if (v0.length > 1e-6 and v1.length > 1e-6) else 0.0
    is_aarm = apex >= APEX_ANGLE_MIN
    return tyre, j0, j1, apex, arm_len, is_aarm

# ---------- safetycell 食い込み距離(健全性確認のみ) ----------
def safetycell_obj():
    if not USE_SAFETYCELL: return None
    o = bpy.data.objects.get(SAFETYCELL_NAME)
    if o and o.type == 'MESH': return o
    for ob in bpy.data.objects:
        if ob.type == 'MESH' and ('safety' in ob.name.lower() or 'cell' in ob.name.lower()):
            return ob
    return None

def penetration(sc, p_world):
    """safetycell 表面までの符号付き距離(負=内部/食い込み)。失敗時 None。"""
    if sc is None: return None
    try:
        pl = sc.matrix_world.inverted() @ p_world
        res = sc.closest_point_on_mesh(pl)
        if not res[0]: return None
        loc, nor = res[1], res[2]
        signed = (pl - loc).dot(nor)        # <0 で内部
        return signed
    except Exception:
        return None

def single_chassis(arm_obj, wc, sc):
    """単リンクの車体側付け根 = 最インボード端(ホイールから最遠の頂点の近傍重心)。
       safetycellは非ウォータータイトで内外判定が誤るため位置決めには使わない。"""
    P = np_pts(arm_obj)
    d = np.linalg.norm(P - np.array([wc.x, wc.y, wc.z]), axis=1)
    far = P[int(d.argmax())]                              # 最インボード端の頂点
    arm_len = float(np.linalg.norm(P.max(0) - P.min(0)))
    near = P[np.linalg.norm(P - far, axis=1) <= 0.20 * arm_len]   # その近傍(=付け根ブッシュ)
    return Vector(near.mean(0)), "tip"

# ---------- ヌル/親子 ----------
def look_basis(pivot, target):
    fwd = (target - pivot)
    if fwd.length < 1e-9: return Matrix.Identity(3)
    fwd.normalize()
    x_axis = -fwd
    up = Vector((0, 0, 1))
    z_axis = x_axis.cross(up)
    if z_axis.length < 1e-6: z_axis = x_axis.cross(Vector((0, 1, 0)))
    z_axis.normalize()
    y_axis = z_axis.cross(x_axis).normalized()
    return Matrix((x_axis, y_axis, z_axis)).transposed()

def car_collection():
    for nm in ["x0_tyre_fl", "wheel_fl", "x0_tyre_bl"]:
        o = bpy.data.objects.get(nm)
        if o and o.users_collection: return o.users_collection[0]
    return bpy.context.scene.collection

def make_empty_m(name, matrix4, size, coll):
    e = bpy.data.objects.get(name)
    if e is None:
        e = bpy.data.objects.new(name, None); coll.objects.link(e)
    elif e.type != 'EMPTY':
        raise RuntimeError("名前衝突: '%s'(%s)" % (name, e.type))
    for c in list(e.users_collection): c.objects.unlink(e)
    coll.objects.link(e)
    e.empty_display_type = 'PLAIN_AXES'; e.empty_display_size = size
    e.parent = None
    e.matrix_world = matrix4
    return e

def set_parent_keep(child, parent):
    bpy.context.view_layer.update()
    w = child.matrix_world.copy()
    child.parent = parent
    child.matrix_parent_inverse.identity()
    child.matrix_world = w

def arm_names(suffix):
    out = []
    for n in ARM_NUMS:
        nm = "suspension_%s" % suffix if n is None else "suspension%d_%s" % (n, suffix)
        if bpy.data.objects.get(nm): out.append(nm)
    return out

def decal_for(arm):
    return bpy.data.objects.get("a_" + arm) if INCLUDE_DECALS else None

def reset_corner(suffix):
    for arm in arm_names(suffix):
        for nm in [arm, "a_" + arm]:
            o = bpy.data.objects.get(nm)
            if o and o.parent:
                w = o.matrix_world.copy(); o.parent = None; o.matrix_world = w
        for nm in [POINTER_PREFIX + arm, "DIR_" + POINTER_PREFIX + arm, arm + "_cjF", arm + "_cjR"]:
            o = bpy.data.objects.get(nm)
            if o and o.type == 'EMPTY': bpy.data.objects.remove(o, do_unlink=True)
    t = bpy.data.objects.get("tether_%s" % suffix)
    if t and t.parent:
        w = t.matrix_world.copy(); t.parent = None; t.matrix_world = w
    bpy.context.view_layer.update()

def pen_str(sc, p):
    s = penetration(sc, p)
    return "" if s is None else " cell=%.3f%s" % (s, "(内)" if s < 0 else "(外)")


def build_corner(corner, sc):
    suffix = CORNER_SUFFIX[corner]
    coll = car_collection()
    wheel = bpy.data.objects.get("WHEEL_%s" % corner)
    hub = bpy.data.objects.get("HUB_%s" % corner)
    if not wheel:
        log("  [SKIP %s] WHEEL_%s 無し" % (corner, corner)); return
    wc = wheel.matrix_world.translation.copy()
    if RESET_FIRST: reset_corner(suffix)
    arms = arm_names(suffix)
    if not arms:
        log("  [%s] arm 無し" % corner); return
    log("  [%s] arms=%s  WHEEL=%s" % (corner, arms, fmt(wc)))

    tether = bpy.data.objects.get("tether_%s" % suffix)
    tc = None
    if tether:
        P = np_pts(tether); tc = Vector((P.min(0) + P.max(0)) * 0.5) if P is not None else None
    arm_recs = []; aarm_info = []

    for arm in arms:
        o = bpy.data.objects.get(arm)
        res = analyze_arm(o, wc)
        if res is None:
            log("    [skip] %s 頂点不足" % arm); continue
        tyre, j0, j1, apex, arm_len, is_aarm = res

        if is_aarm:
            cjF, cjR = sorted([j0, j1], key=lambda j: j.y)   # 前=-Y
            pivot = (cjF + cjR) * 0.5
            psrc = "mid(cjF,cjR)"
        else:
            cjF = cjR = None
            pivot, psrc = single_chassis(o, wc, sc)   # 車体側付け根(cell食い込み or 最インボード端)

        sz = max((pivot - tyre).length * SIZE_FACTOR, 1e-4)
        pname = POINTER_PREFIX + arm; dname = "DIR_" + pname
        Rp = look_basis(pivot, tyre).to_4x4()
        pointer = make_empty_m(pname, Matrix.Translation(pivot) @ Rp, sz, coll)
        dirnull = make_empty_m(dname, Matrix.Translation(tyre), sz * 0.6, coll)
        if is_aarm:
            make_empty_m(arm + "_cjF", Matrix.Translation(cjF) @ AC_BASE, sz * 0.5, coll)
            make_empty_m(arm + "_cjR", Matrix.Translation(cjR) @ AC_BASE, sz * 0.5, coll)
        bpy.context.view_layer.update()
        if DIR_PARENT_TO_HUB and hub: set_parent_keep(dirnull, hub)
        set_parent_keep(o, pointer)
        dec = decal_for(arm)
        if dec: set_parent_keep(dec, pointer)
        arm_recs.append((arm, pointer, o))

        if is_aarm:
            aarm_info.append((arm, (cjF.z + cjR.z) * 0.5))
            log("    %s: [A字] apex=%.1f° len=%.3f  cjF=%s%s cjR=%s%s  tyre=%s%s"
                % (arm, apex, arm_len, fmt(cjF), pen_str(sc, cjF), fmt(cjR), pen_str(sc, cjR),
                   fmt(tyre), " +decal" if dec else ""))
        else:
            log("    %s: [単] apex=%.1f° len=%.3f  car(%s)=%s%s  tyre=%s%s"
                % (arm, apex, arm_len, psrc, fmt(pivot), pen_str(sc, pivot), fmt(tyre),
                   " +decal" if dec else ""))

    if tether and tc and arm_recs:
        inside = None; best = None; bestd = 1e18
        for arm, pointer, o in arm_recs:
            P = np_pts(o); mn = Vector(P.min(0)); mx = Vector(P.max(0))
            if all(mn[i] <= tc[i] <= mx[i] for i in range(3)):
                inside = (arm, pointer); break
            c = (mn + mx) * 0.5; dd = (c - tc).length
            if dd < bestd: bestd = dd; best = (arm, pointer)
        ch = inside or best
        if ch:
            set_parent_keep(tether, ch[1])
            log("    tether_%s -> %s (%s)" % (suffix, ch[0], "包含" if inside else "最近接"))

    if aarm_info:
        aarm_info.sort(key=lambda t: t[1])
        log("    [hint] A字 低→高: %s  →最上=アッパー候補/最下=ロア候補"
            % " ; ".join("%s(z=%.3f)" % (a, z) for a, z in aarm_info))


def main():
    log("============ 操作6: DIR_ + ハードポイント (開き角A字判定) ============")
    sc = safetycell_obj()
    log("PROCESS_CORNERS=%s  APEX_ANGLE_MIN=%.0f°  safetycell=%s"
        % (PROCESS_CORNERS, APEX_ANGLE_MIN, sc.name if sc else "(無効/無し)"))
    for c in PROCESS_CORNERS:
        build_corner(c, sc)
    bpy.context.view_layer.update()
    log("====================================================================")
    print("\n\n##### COPY FROM HERE / ここから下を丸ごと貼ってください #####")
    print("\n".join(REPORT))
    print("##### COPY UNTIL HERE / ここまで #####")

if __name__ == "__main__":
    main()
