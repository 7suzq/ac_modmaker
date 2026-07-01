# -*- coding: utf-8 -*-
"""
AC mod 一括ビルド (操作1〜5) ＋ 自動整頓  —— これ1本でOK
================================================================
・操作1 接地Z調整 → 操作2/3 前 → 操作4 後 → 操作5 ステア
・エンプティは「車オブジェクト(タイヤ等)があるコレクションを自動検出」してそこに作成
・エンプティ表示サイズはタイヤ/ステア基準で自動(線がタイヤ外まで伸びる)
・COCKPIT_HR の高さ(Z)は halo の最下面。X=0, Y=-0.16m。
階層(AC公式): SUSP_XX > HUB_XX(caliper) > WHEEL_XX(tyre,rim,disc) / COCKPIT_HR > STEER_HR
実行: Scripting で Run → 末尾の COPY ブロックを返信に貼ってください。
"""

import bpy
from mathutils import Vector, Matrix, kdtree
from math import radians

# ============================== CONFIG ==============================
TYRE_NAMES = ["x0_tyre_fl", "x0_tyre_fr", "x0_tyre_bl", "x0_tyre_br"]
SINK_MM = 0.003                 # 操作1: タイヤ底を地面より沈める量(実寸mm)。後で調整可。
COCKPIT_Y_METERS = -0.16        # 操作5: COCKPIT_HR の Y(実寸m)

WHEEL_SIZE_FACTOR = 0.7         # ホイール系ヌル表示サイズ = タイヤ最大寸 × これ
STEER_SIZE_FACTOR = 0.7         # STEER/COCKPIT 表示サイズ = steering_wheel最大寸 × これ

HALO_NAME = ""                  # 空なら自動検出("hal"を含むメッシュ, デカールa_*は除外)。明示するなら名前を入れる。

# エンプティの向き:
#   "ac"       : Blenderで X=+90°。標準FBX書き出し(Z-up→Y-up = X軸-90°回転)後に
#                AC内で「Z前/Y上」(=AC座標で identity)になる。 ← AC公式ルール準拠
#   "identity" : Blender軸に整列(回転なし)。書き出しがZ-up→Y-up変換をしない場合はこちら。
# ※ どちらでもホイールのスピン軸(=車のX/左右=アクスル)は保たれるので回転は正しく出ます。
#    違いはY/Z軸の向きで、transmission/ARROW_/DIR_(操作6以降)やAC公式ルールに効きます。
# ※ あなたのFBX書き出し方法が「標準(Z-up→Y-up)」なら "ac" が正解です。要確認。
EMPTY_ORIENT = "ac"

CORNERS = {  # rear: LR=_bl, RR=_br
    "LF": dict(tyre="x0_tyre_fl", rim="wheel_fl", disc="disc_fl", caliper="hub_caliper_fl"),
    "RF": dict(tyre="x0_tyre_fr", rim="wheel_fr", disc="disc_fr", caliper="hub_caliper_fr"),
    "LR": dict(tyre="x0_tyre_bl", rim="wheel_bl", disc="disc_bl", caliper="hub_caliper_bl"),
    "RR": dict(tyre="x0_tyre_br", rim="wheel_br", disc="disc_br", caliper="hub_caliper_br"),
}
STEER_MESHES = ["steering_wheel", "left_paddle", "right_paddle", "mix_dial", "tyre_dial",
                "a_steering_wheel", "a_left_paddle", "a_right_paddle", "a_mix_dial", "a_tyre_dial"]
STEER_WHEEL_REF = "steering_wheel"
COCKPIT_REF     = "cockpit"

SAVE_AS = None                  # 別名保存したいなら r"C:\path\out.blend"
RESET_BEFORE_BUILD = True        # True: 毎回クリーンに組み直す(既存の管理ヌル削除+対象メッシュを親なしに戻す)。再実行安全。
# ===================================================================

CAR_COLL = None   # main で自動検出
REPORT = []
def log(s): REPORT.append(s); print(s)

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

def aabb_center(names):
    mn = Vector((1e18,)*3); mx = Vector((-1e18,)*3); found = []
    for n in (names if isinstance(names,(list,tuple)) else [names]):
        o = bpy.data.objects.get(n)
        if not o: continue
        pts = world_verts(o)
        if not pts: continue
        found.append(n)
        for p in pts:
            mn.x=min(mn.x,p.x); mn.y=min(mn.y,p.y); mn.z=min(mn.z,p.z)
            mx.x=max(mx.x,p.x); mx.y=max(mx.y,p.y); mx.z=max(mx.z,p.z)
    return ((mn+mx)*0.5 if found else None), found

def fmt(v): return "(%.4f, %.4f, %.4f)" % (v.x, v.y, v.z)

def find_car_collection():
    """タイヤ/ホイールが入っているコレクションを自動検出。"""
    for nm in TYRE_NAMES + ["wheel_fl","wheel_fr","wheel_bl","wheel_br"]:
        o = bpy.data.objects.get(nm)
        if o and o.users_collection:
            return o.users_collection[0]
    return bpy.context.scene.collection

def find_halo():
    """名前に'hal'を含むメッシュを自動検出(デカールa_*除外, 完全一致'hal'優先)。"""
    if HALO_NAME:
        o = bpy.data.objects.get(HALO_NAME)
        if o: return o
    cands = [o for o in bpy.data.objects
             if o.type == 'MESH' and 'hal' in o.name.lower()
             and not o.name.lower().startswith('a_')]
    cands.sort(key=lambda o: (o.name.lower() != 'hal', len(o.name)))
    return cands[0] if cands else None

def make_empty(name, loc, size):
    coll = CAR_COLL or bpy.context.scene.collection
    e = bpy.data.objects.get(name)
    if e is None:
        e = bpy.data.objects.new(name, None)
        coll.objects.link(e)
    elif e.type != 'EMPTY':
        raise RuntimeError("名前衝突: '%s'(%s)" % (name, e.type))
    for c in list(e.users_collection):   # 目的コレクションへ入れ直す
        c.objects.unlink(e)
    coll.objects.link(e)
    e.empty_display_type = 'PLAIN_AXES'; e.empty_display_size = size
    e.parent = None
    basis = Matrix.Rotation(radians(90), 4, 'X') if EMPTY_ORIENT == "ac" else Matrix.Identity(4)
    e.matrix_world = Matrix.Translation(loc) @ basis
    return e

def set_parent_keep(child, parent):
    bpy.context.view_layer.update()
    w = child.matrix_world.copy()
    child.parent = parent
    child.matrix_parent_inverse.identity()
    child.matrix_world = w

def reparent(empty, names):
    done = []
    for n in (names if isinstance(names,(list,tuple)) else [names]):
        o = bpy.data.objects.get(n)
        if o: set_parent_keep(o, empty); done.append(n)
    return done


def managed_null_names():
    ns = []
    for c in CORNERS:
        ns += ["SUSP_%s" % c, "HUB_%s" % c, "WHEEL_%s" % c]
    ns += ["COCKPIT_HR", "STEER_HR"]
    return ns

def managed_mesh_names():
    s = set()
    for m in CORNERS.values():
        s.update([m["tyre"], m["rim"], m["disc"], m["caliper"]])
    s.update(STEER_MESHES)
    return s

def reset_previous():
    """再実行安全化: 対象メッシュをworld保持で親なしに戻し、管理ヌルを削除。
       (向き変更時に子メッシュが引きずられる問題を回避。DIR_等の他ヌルには触れない)"""
    for nm in managed_mesh_names():
        o = bpy.data.objects.get(nm)
        if o and o.parent is not None:
            w = o.matrix_world.copy()
            o.parent = None
            o.matrix_world = w
    bpy.context.view_layer.update()
    removed = 0
    for nm in managed_null_names():
        o = bpy.data.objects.get(nm)
        if o and o.type == 'EMPTY':
            bpy.data.objects.remove(o, do_unlink=True); removed += 1
    bpy.context.view_layer.update()
    return removed


# ------------------------------ 操作1 ------------------------------
def op1_zfit():
    tyres = [bpy.data.objects.get(n) for n in TYRE_NAMES]
    miss = [n for n,o in zip(TYRE_NAMES,tyres) if not o]
    tyres = [o for o in tyres if o]
    if miss: log("  [WARN] tyre not found: %s" % miss)
    if not tyres: log("  [ERROR] no tyres -> skip op1"); return
    zmin = min(min(p.z for p in world_verts(o)) for o in tyres if world_verts(o))
    sl = scale_length(); sink_bu = (SINK_MM/1000.0)/sl
    delta = (-sink_bu) - zmin
    moved = 0
    for o in bpy.data.objects:
        if o.parent is None:
            o.location.z += delta; moved += 1
    bpy.context.view_layer.update()
    zmin2 = min(min(p.z for p in world_verts(o)) for o in tyres if world_verts(o))
    log("  scale_length(m/BU)=%s" % sl)
    log("  lowest tyre Z before=%.6f  shift=%.6f (to %d roots)  after=%.8f (target %.8f)"
        % (zmin, delta, moved, zmin2, -sink_bu))


# ---------------------------- 操作2/3/4 ----------------------------
def op_corner(c):
    m = CORNERS[c]
    center, found = aabb_center([m["tyre"], m["rim"], m["disc"]])
    if center is None:
        log("  [SKIP] %s: tyre/rim/disc not found" % c); return
    to = bpy.data.objects.get(m["tyre"])
    sz = (max(to.dimensions) * WHEEL_SIZE_FACTOR) if to else 0.5
    susp  = make_empty("SUSP_%s"  % c, center, sz)
    hub   = make_empty("HUB_%s"   % c, center, sz)
    wheel = make_empty("WHEEL_%s" % c, center, sz)
    bpy.context.view_layer.update()
    set_parent_keep(hub, susp); set_parent_keep(wheel, hub)
    bpy.context.view_layer.update()
    cal  = reparent(hub,   m["caliper"])
    spin = reparent(wheel, [m["tyre"], m["rim"], m["disc"]])
    log("  [%s] center=%s size=%.4f  HUB<-%s  WHEEL<-%s" % (c, fmt(center), sz, cal, spin))


# ------------------------------ 操作5 ------------------------------
def op5_steering():
    sw = bpy.data.objects.get(STEER_WHEEL_REF)
    if not sw: log("  [SKIP] steering: '%s' not found" % STEER_WHEEL_REF); return
    pts = world_verts(sw)
    sw_min_y = min(p.y for p in pts); sw_max_z = max(p.z for p in pts)
    sw_center,_ = aabb_center([STEER_WHEEL_REF])

    # STEER_HR Z = steering_wheel と cockpit の接触円中心Z
    steer_z = sw_center.z; contact_txt = "cockpit無し→sw中心Z代用"
    cp = bpy.data.objects.get(COCKPIT_REF)
    if cp:
        b = world_verts(cp)
        if b:
            kd = kdtree.KDTree(len(b))
            for i,p in enumerate(b): kd.insert(p, i)
            kd.balance()
            d = sorted(((kd.find(p)[2], p) for p in pts), key=lambda t: t[0])
            k = max(20, int(len(d)*0.02)); cc = Vector((0,0,0))
            for _,p in d[:k]: cc += p
            cc /= k; steer_z = cc.z
            contact_txt = "contact_center=%s d_min=%.5f" % (fmt(cc), d[0][0])

    # COCKPIT_HR Z = halo 最下面
    halo = find_halo()
    if halo and world_verts(halo):
        cockpit_z = min(p.z for p in world_verts(halo))
        halo_txt = "halo='%s' bottomZ=%.5f" % (halo.name, cockpit_z)
    else:
        cockpit_z = sw_max_z
        halo_txt = "halo未検出→COCKPIT Z は steering_wheel上面で代用(%.5f)" % sw_max_z

    sl = scale_length()
    ssz = max(sw.dimensions) * STEER_SIZE_FACTOR
    steer_loc   = Vector((0.0, sw_min_y, steer_z))
    cockpit_loc = Vector((0.0, COCKPIT_Y_METERS/sl, cockpit_z))
    steer   = make_empty("STEER_HR",   steer_loc, ssz)
    cockpit = make_empty("COCKPIT_HR", cockpit_loc, ssz)
    bpy.context.view_layer.update()
    set_parent_keep(steer, cockpit)
    got = reparent(steer, STEER_MESHES)
    log("  sw_center=%s sw_min_y=%.5f sw_max_z=%.5f" % (fmt(sw_center), sw_min_y, sw_max_z))
    log("  STEER_HR: %s" % contact_txt)
    log("  COCKPIT_HR: %s" % halo_txt)
    log("  STEER_HR   loc=%s size=%.4f" % (fmt(steer_loc), ssz))
    log("  COCKPIT_HR loc=%s (Y=%.3fm=%.5fBU)" % (fmt(cockpit_loc), COCKPIT_Y_METERS, COCKPIT_Y_METERS/sl))
    log("  STEER_HR <- %s" % got)


# ------------------------------ main ------------------------------
def main():
    global CAR_COLL
    CAR_COLL = find_car_collection()
    log("================ AC mod build (op1-5, auto) ================")
    log("car collection (auto) = '%s'   EMPTY_ORIENT=%s" % (CAR_COLL.name, EMPTY_ORIENT))
    if RESET_BEFORE_BUILD:
        n = reset_previous(); log("reset: removed %d managed nulls, detached target meshes" % n)
    log("-- 操作1: Z fit --");      op1_zfit()
    log("-- 操作2/3: front --");    op_corner("LF"); op_corner("RF")
    log("-- 操作4: rear --");       op_corner("LR"); op_corner("RR")
    log("-- 操作5: steering --");   op5_steering()
    bpy.context.view_layer.update()
    log("===========================================================")
    if SAVE_AS:
        bpy.ops.wm.save_as_mainfile(filepath=SAVE_AS); log("saved -> %s" % SAVE_AS)
    print("\n\n##### COPY FROM HERE / ここから下を丸ごとコピーして返信に貼ってください #####")
    print("\n".join(REPORT))
    print("##### COPY UNTIL HERE / ここまで #####")

if __name__ == "__main__":
    main()
