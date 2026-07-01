# -*- coding: utf-8 -*-
"""
操作8a: フロントウイング Wing flex 自動セットアップ (rev5)
================================================================================
rev5の追加:
 【縦しなり(vert)】ノーズ接合部(frontspoiler_fm 後端・中央)にピッチボーンを1本置き、
   左右スパン鎖の"親"にする。ピッチボーンをワールドX軸まわりに回すと、
   ノーズ接合部を軸に前方(風を受ける側 -Y)ほど大きくZへ動く。
   親子なのでスパン鎖(横)と自然に合成される。
 【モード切替】FLEX_MODE = "both" / "vertical" / "horizontal"

rev4までの機能:
 位置ベース幾何ウェイト(浮いたディテールも追従)、デカール浮かし切替、
 全メッシュ Merge by Distance(除外可)、親付け不具合修正。

しなり(横): ワールドY軸まわり、外側(タイヤ側)ほど下へ。中心固定。
"""
import bpy, os
from mathutils import Vector, Quaternion
from math import radians

# ============================== CONFIG ==============================
FLEX_MODE = "both"          # "both" / "vertical"(縦のみ) / "horizontal"(横のみ)

# --- 横(span) ---
FLEX_SCALE = 0.3            # 横しなり倍率
FLEX_DEG_PER_STEP = 1.2     # 外側へ1節ごとの基準角(度)
DROP_SIGN = +1.0            # +1=タイヤ側が下へ / -1=上へ

# --- 縦(vert=ピッチ) ---
VERT_SCALE = 0.3            # 縦しなり倍率
VERT_DEG = 2.5             # ピッチ基準角(度)。実角 = VERT_DEG*VERT_SCALE
VERT_SIGN = -1.0           # +1/-1 で前方が下/上(逆なら反転)
PITCH_PIVOT_Y = None       # ノーズ接合部Yを手動指定(None=自動: frontspoiler_fm後端中央)

DECAL_FLOATING = 1
DECAL_OFFSET = 0.0004
MERGE_ALL = 1
MERGE_DIST = 0.0001
MERGE_EXCLUDE = []

WING_PARTS = [
    "frontspoiler_fm",
    "frontwingmoving_fl", "frontwingmoving_fr",
    "frontwing_fl_01", "frontwing_fr_01",
]
DECAL_PREFIX = "a_"
ARMATURE_NAME = "fw_armature"
ANIM_END_FRAME = 100
WING_INDEX = 9
KSANIM_NAME = "fw_flex.ksanim"
OUT_TXT = "_wingflex_inis.txt"
CHAIN_FRACS = [0.0, 0.25, 0.5, 0.75, 0.97]
PITCH_BONE = "Bone_pitch"
RESET = True
# ===================================================================

BONES_L = ["Bone", "Bone001", "Bone002", "Bone003"]
BONES_R = ["Bone004", "Bone005", "Bone006", "Bone007"]
BONES_SPAN = BONES_L + BONES_R
BONES_ALL = [PITCH_BONE] + BONES_SPAN

REPORT = []
def log(s): REPORT.append(s); print(s)

def world_bb(o): return [o.matrix_world @ Vector(c) for c in o.bound_box]
def world_aabb(objs):
    mn = Vector((1e18,)*3); mx = Vector((-1e18,)*3)
    for o in objs:
        for p in world_bb(o):
            mn.x=min(mn.x,p.x); mn.y=min(mn.y,p.y); mn.z=min(mn.z,p.z)
            mx.x=max(mx.x,p.x); mx.y=max(mx.y,p.y); mx.z=max(mx.z,p.z)
    return mn, mx

def car_collection():
    for nm in ["x0_tyre_fl", "wheel_fl"]:
        o = bpy.data.objects.get(nm)
        if o and o.users_collection: return o.users_collection[0]
    return bpy.context.scene.collection

def base_parts():
    return [bpy.data.objects[nm] for nm in WING_PARTS
            if bpy.data.objects.get(nm) and bpy.data.objects[nm].type == 'MESH']

def decal_of(base):
    d = bpy.data.objects.get(DECAL_PREFIX + base.name)
    return d if (d and d.type == 'MESH') else None

def nose_joint(fm, cx, half, cz):
    """frontspoiler_fm の中央付近で最も後方(+Y最大)の頂点 = ノーズ接合部。"""
    mw = fm.matrix_world
    best = None
    for v in fm.data.vertices:
        p = mw @ v.co
        if abs(p.x - cx) < 0.10 * max(half, 1e-4):
            if best is None or p.y > best.y:
                best = p
    if best is None:
        mn, mx = world_aabb([fm])
        best = Vector((cx, mx.y, cz))
    return best.copy()

# ---------- Merge by Distance(全メッシュ) ----------
def merge_all():
    done = 0; seen = set(); prev = bpy.context.view_layer.objects.active
    for o in list(bpy.data.objects):
        if o.type != 'MESH' or o.name in MERGE_EXCLUDE: continue
        if o.data.name in seen: continue
        seen.add(o.data.name)
        hid = o.hide_get()
        try:
            o.hide_set(False)
            bpy.ops.object.select_all(action='DESELECT')
            bpy.context.view_layer.objects.active = o; o.select_set(True)
            bpy.ops.object.mode_set(mode='EDIT')
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.remove_doubles(threshold=MERGE_DIST)
            bpy.ops.object.mode_set(mode='OBJECT')
            done += 1
        except Exception as e:
            log("  [merge skip] %s: %s" % (o.name, e))
            try: bpy.ops.object.mode_set(mode='OBJECT')
            except Exception: pass
        finally:
            o.hide_set(hid)
    bpy.context.view_layer.objects.active = prev
    log("Merge by Distance(%.4f): %d メッシュ (除外 %d)" % (MERGE_DIST, done, len(MERGE_EXCLUDE)))

# ---------- デカール法線オフセット ----------
def bake_decal_offset(decal):
    if decal.get("fw_offset"): return
    me = decal.data
    if me.users > 1: decal.data = me.copy(); me = decal.data
    for v in me.vertices: v.co = v.co + v.normal * DECAL_OFFSET
    me.update(); decal["fw_offset"] = DECAL_OFFSET

def unbake_decal_offset(decal):
    off = decal.get("fw_offset")
    if not off: return
    me = decal.data
    for v in me.vertices: v.co = v.co - v.normal * off
    me.update(); del decal["fw_offset"]

def reset_prior():
    o = bpy.data.objects.get(ARMATURE_NAME)
    if o:
        dat = o.data if (o.data and o.data.users == 1) else None
        bpy.data.objects.remove(o, do_unlink=True)
        if dat: bpy.data.armatures.remove(dat)
    for b in base_parts():
        targets = [b]; d = decal_of(b)
        if d: targets.append(d)
        for ob in targets:
            for m in [m for m in ob.modifiers if m.type == 'ARMATURE']:
                ob.modifiers.remove(m)
            if ob.parent and ob.parent.type == 'ARMATURE':
                w = ob.matrix_world.copy(); ob.parent = None; ob.matrix_world = w
        if d: unbake_decal_offset(d)

def build_armature(parts, pivot):
    mn, mx = world_aabb(parts)
    cx = (mn.x + mx.x) * 0.5; cy = (mn.y + mx.y) * 0.5; cz = (mn.z + mx.z) * 0.5
    half = (mx.x - mn.x) * 0.5; chord = (mx.y - mn.y)
    arm = bpy.data.armatures.new(ARMATURE_NAME)
    obj = bpy.data.objects.new(ARMATURE_NAME, arm)
    car_collection().objects.link(obj)
    obj.location = Vector((0, 0, 0))   # ワールド原点(=ワールド軸)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    eb = arm.edit_bones
    # ピッチボーン(ノーズ接合部 → 前方)
    pb = eb.new(PITCH_BONE)
    pb.head = pivot.copy()
    pb.tail = pivot + Vector((0, -max(chord*0.25, 0.05), 0))
    # スパン鎖(中心→翼端)。ルートをピッチボーンの子に。
    def chain(prefix, sign):
        prev = None
        for i in range(len(CHAIN_FRACS) - 1):
            b = eb.new(prefix + str(i))
            b.head = Vector((cx + sign*CHAIN_FRACS[i]  *half, cy, cz))
            b.tail = Vector((cx + sign*CHAIN_FRACS[i+1]*half, cy, cz))
            b.parent = prev if prev else pb
            b.use_connect = bool(prev)
            prev = b
    chain("L", +1.0); chain("R", -1.0)
    bpy.ops.object.mode_set(mode='OBJECT')
    ren = {"L0":"Bone","L1":"Bone001","L2":"Bone002","L3":"Bone003",
           "R0":"Bone004","R1":"Bone005","R2":"Bone006","R3":"Bone007"}
    for old, new in ren.items():
        if old in arm.bones: arm.bones[old].name = new
    return obj, cx

# ---------- 位置ベース幾何ウェイト(横スパン) ----------
def side_anchors(arm_obj, names, cx):
    lst = []
    for nm in names:
        b = arm_obj.data.bones.get(nm)
        if not b: continue
        mid = arm_obj.matrix_world @ ((b.head_local + b.tail_local) * 0.5)
        lst.append((nm, abs(mid.x - cx)))
    lst.sort(key=lambda a: a[1]); return lst

def blend2(anchors, d):
    if d <= anchors[0][1]: return (anchors[0][0], 1.0), None
    if d >= anchors[-1][1]: return (anchors[-1][0], 1.0), None
    for i in range(len(anchors)-1):
        d0 = anchors[i][1]; d1 = anchors[i+1][1]
        if d0 <= d <= d1:
            f = (d-d0)/(d1-d0) if d1 > d0 else 0.0
            return (anchors[i][0], 1.0-f), (anchors[i+1][0], f)
    return (anchors[-1][0], 1.0), None

def fresh_groups(obj):
    for nm in BONES_SPAN:
        vg = obj.vertex_groups.get(nm)
        if vg: obj.vertex_groups.remove(vg)
    for nm in BONES_SPAN:
        obj.vertex_groups.new(name=nm)

def assign_geo_weights(obj, anchL, anchR, cx):
    fresh_groups(obj)
    gi = {vg.name: vg.index for vg in obj.vertex_groups}
    mw = obj.matrix_world
    for v in obj.data.vertices:
        x = (mw @ v.co).x
        anchors = anchL if x >= cx else anchR
        (n0, w0), second = blend2(anchors, abs(x - cx))
        obj.vertex_groups[gi[n0]].add([v.index], w0, 'REPLACE')
        if second:
            obj.vertex_groups[gi[second[0]]].add([v.index], second[1], 'REPLACE')

def parent_keep(obj, arm_obj):
    if not any(m.type == 'ARMATURE' for m in obj.modifiers):
        m = obj.modifiers.new("Armature", 'ARMATURE'); m.object = arm_obj
    else:
        for m in obj.modifiers:
            if m.type == 'ARMATURE': m.object = arm_obj
    obj.parent = arm_obj
    obj.matrix_parent_inverse = arm_obj.matrix_world.inverted()

def keyframe_flex(arm_obj):
    do_h = FLEX_MODE in ("both", "horizontal")
    do_v = FLEX_MODE in ("both", "vertical")
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='POSE')
    pb = arm_obj.pose.bones
    # rest
    bpy.context.scene.frame_set(1)
    for n in BONES_ALL:
        b = pb.get(n)
        if not b: continue
        b.rotation_mode = 'QUATERNION'; b.rotation_quaternion = (1,0,0,0)
        b.keyframe_insert("rotation_quaternion", frame=1)
    bpy.context.scene.frame_set(ANIM_END_FRAME)
    def rot_world(bone, axis_world, ang):
        axis_local = bone.bone.matrix_local.to_3x3().inverted() @ axis_world
        if axis_local.length < 1e-9: return
        bone.rotation_quaternion = Quaternion(axis_local.normalized(), ang)
        bone.keyframe_insert("rotation_quaternion", frame=ANIM_END_FRAME)
    # 縦(ピッチ): ワールドX軸
    bp = pb.get(PITCH_BONE)
    if bp:
        ang = radians(VERT_DEG * VERT_SCALE) * VERT_SIGN if do_v else 0.0
        rot_world(bp, Vector((1,0,0)), ang)
    # 横(スパン): ワールドY軸、外側ほど大きく、左右反転
    def bend(names, side_sign):
        for i, n in enumerate(names):
            b = pb.get(n)
            if not b: continue
            ang = radians(i*FLEX_DEG_PER_STEP*FLEX_SCALE)*side_sign*DROP_SIGN if do_h else 0.0
            rot_world(b, Vector((0,1,0)), ang)
    bend(BONES_L, +1.0); bend(BONES_R, -1.0)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.scene.frame_set(1)

def ini_text():
    aero = ("[WING_%d]\nNAME=fw_flex\nCHORD=1\nSPAN=1\nPOSITION=0,-0.013,2.762\n"
            "LUT_AOA_CL=Neutral.lut\nLUT_GH_CL=Neutral.lut\nCL_GAIN=0\n"
            "LUT_AOA_CD=Neutral.lut\nLUT_GH_CD=Neutral.lut\nCD_GAIN=0\nANGLE=0\n"
            "ZONE_FRONT_CL=0\nZONE_FRONT_CD=0\nZONE_REAR_CL=0\nZONE_REAR_CD=0\n"
            "ZONE_LEFT_CL=0\nZONE_LEFT_CD=0\nZONE_RIGHT_CL=0\nZONE_RIGHT_CD=0\nYAW_CL_GAIN=0\n\n"
            "[DYNAMIC_CONTROLLER_0]\nWING=%d\nCOMBINATOR=ADD\nINPUT=SPEED_KMH\n"
            "LUT=fw_flexanim.lut\nFILTER=0.9\nUP_LIMIT=100\nDOWN_LIMIT=0\n" % (WING_INDEX, WING_INDEX))
    wa = "[ANIMATION_0]\nWING=%d\nFILE=%s\nMIN=0\nMAX=100\n" % (WING_INDEX, KSANIM_NAME)
    def wob(i, name):
        return ("[WOBBLY_BIT_%d]\nNAME = %s\nCONNECTED_TO = 0, 0, 0\nMAX_RANGE = 0.10\n"
                "DAMPENING_LAG = 0.93\nG_GAIN = 0.7\nGRAVITY_GAIN = 1.0\nOFFSET_GAIN = 1500\n"
                "STIFF_AXIS = 0, 0, 1\nSTIFF_AXIS_STIFFNESS = 0.15\nG_FILTER = 0.0\n"
                "DEFAULT_GRAVITY_INCLUDED_ALREADY = 1\n" % (i, name))
    return aero, wa, wob(0, "Bone003") + "\n" + wob(1, "Bone007")

def main():
    log("============ 操作8a: Wing flex rev5 ============")
    log("FLEX_MODE=%s  横:FLEX_SCALE=%.2f DROP=%+d  縦:VERT_SCALE=%.2f VERT_SIGN=%+d"
        % (FLEX_MODE, FLEX_SCALE, int(DROP_SIGN), VERT_SCALE, int(VERT_SIGN)))
    bases = base_parts()
    if not bases:
        log("[ERROR] ウイングパーツ無し"); return
    if RESET: reset_prior()
    if MERGE_ALL: merge_all()

    mn, mx = world_aabb(bases)
    cx0 = (mn.x+mx.x)*0.5; cz0 = (mn.z+mx.z)*0.5; half0 = (mx.x-mn.x)*0.5
    fm = bpy.data.objects.get("frontspoiler_fm")
    pivot = nose_joint(fm, cx0, half0, cz0) if fm else Vector((cx0, mx.y, cz0))
    if PITCH_PIVOT_Y is not None: pivot.y = PITCH_PIVOT_Y
    log("ノーズ接合部(ピッチ軸) = (%.3f, %.3f, %.3f)" % (pivot.x, pivot.y, pivot.z))

    arm_obj, cx = build_armature(bases, pivot)
    bpy.context.view_layer.update()
    anchL = side_anchors(arm_obj, BONES_L, cx)
    anchR = side_anchors(arm_obj, BONES_R, cx)
    log("Armature生成: ピッチ親 + 中心→翼端の4+4鎖")

    for b in bases:
        assign_geo_weights(b, anchL, anchR, cx)
        parent_keep(b, arm_obj)
        d = decal_of(b)
        if d:
            assign_geo_weights(d, anchL, anchR, cx)
            parent_keep(d, arm_obj)
            if DECAL_FLOATING: bake_decal_offset(d)
            log("  %s + %s(%s)" % (b.name, d.name, "浮かす" if DECAL_FLOATING else "浮かさない"))
        else:
            log("  %s (デカール無し)" % b.name)
    keyframe_flex(arm_obj)
    log("アニメ frame1=rest, frame%d=しなり (mode=%s)" % (ANIM_END_FRAME, FLEX_MODE))

    aero, wa, ext = ini_text()
    full = ("; ===== aero.ini に追記 =====\n%s\n; ===== wing_animation.ini =====\n%s\n"
            "; ===== ext_config.ini に追記 =====\n%s" % (aero, wa, ext))
    blend = bpy.data.filepath
    if blend:
        path = os.path.join(os.path.dirname(blend), OUT_TXT)
        try:
            with open(path, "w", encoding="utf-8") as f: f.write(full)
            log("ini追記用テキスト: %s" % path)
        except Exception as e:
            log("[WARN] 書き出し失敗: %s" % e)
    else:
        log("[WARN] .blend未保存。保存後再実行でファイル出力。")

    log("調整: FLEX_MODE(both/vertical/horizontal), 横=FLEX_SCALE/DROP_SIGN, 縦=VERT_SCALE/VERT_SIGN")
    log("残作業: 原点 Y-up(R+X+90) → FBX → KsEditorで%s → lutをdataへ" % KSANIM_NAME)
    log("================================================")
    print("\n##### COPY (レポート) #####")
    print("\n".join(REPORT))
    print("\n##### ↓ ini追記用テキスト ↓ #####")
    print(full)
    print("##### ここまで #####")

if __name__ == "__main__":
    main()
