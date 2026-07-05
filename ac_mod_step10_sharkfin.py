# -*- coding: utf-8 -*-
"""
操作10: シャークフィン(sharkfin)検出＋左右しなり (rev3)
================================================================================
rev3の修正:
 【境界固定】topcanopy_bm 表面に近接するフィン頂点(=切断シーム)を検出してルートに
   完全ロック(Z下端バンドではなく実シームを固定)→境界は絶対に動かない。
 【後方だけ動かす】ボーン鎖を Y方向(前→後)に張り、ワールドZ軸で yaw(首振り)。
   前=ルート固定/後=振れる。ウェイトも Y位置で配分→中央は動かずリア端だけ左右へ。
 【左右対称】速度ksanimは片方向のみ。アンテナと同じく WOBBLY_BIT を出力し、
   実機では静止=中心のまま左右対称に揺らす(プレビュー用キーフレームは片側)。

安全: DETECT_ONLY=True で検出のみ。失敗時は File>Revert。
"""
import bpy, os, bmesh
from mathutils import Vector, Quaternion
from mathutils.kdtree import KDTree
from math import radians

# ============================== CONFIG ==============================
DETECT_ONLY = False

TOPCANOPY = "topcanopy_bm"
REARWING = "rearwing_top"
NUMBER_TOKEN = "_number"
DECAL_TOKENS = ["_$$3DSimED$$_", "a_topcanopy_bm"]

# 検出
X_SLAB      = 0.005
FIN_GROW    = 0.6
FIN_MIN_Z   = 0.10
NUM_REGION_PAD = 0.15
DECAL_PAD   = 0.02

# 自動検出(番号に依存せず、センターラインの薄い縦板の連結成分から一番背の高い物を採用)
AUTO_DETECT = True
X_SLAB_CANDIDATES = [0.005, 0.008, 0.012, 0.02, 0.03, 0.05, 0.08, 0.12]  # 段階的に薄さを緩める
FIN_MIN_FACES = 4          # 連結成分の最小面数(小さめのフィンも拾う)
FIN_CENTER_X  = 0.0        # フィンのX中心(車体センターライン=0)

# 境界固定 / しなり配分
SEAM_EPS    = 0.0015        # topcanopy_bm 頂点にこの距離以内=シーム→完全固定(m)
FRONT_LOCK_RATIO = 0.15     # 前方この割合(Y)まではルート固定(0=なし)
SF_BONES    = 4             # Y方向の鎖(前=ルート固定, 後=振れる)

# アニメ
ANIM_END_FRAME = 100
SF_BEND_SCALE = 0.02
SF_BEND_DEG_PER_STEP = 4.0
SF_BEND_SIGN = +1.0
SF_ARM = "sharkfin_armature"
SF_WING_INDEX = 11
SF_KSANIM = "sharkfin_flex.ksanim"

# 実機の左右対称揺れ(WOBBLY_BIT)
WOBBLE = dict(MAX_RANGE=0.15, DAMPENING_LAG=0.90, G_GAIN=0.9, GRAVITY_GAIN=1.0,
              OFFSET_GAIN=2000, STIFF_AXIS="0, 0, 1", STIFF="0.10", G_FILTER="0.0")

DECAL_FLOATING = 1
DECAL_OFFSET = 0.0004
OUT_TXT = "_sharkfin_inis.txt"
RESET = True
# ===================================================================

SF_BONES_NAMES = ["SfBone"] + ["SfBone%03d" % i for i in range(1, SF_BONES)]
REPORT = []
def log(s): REPORT.append(s); print(s)

def mesh_obj(name):
    o = bpy.data.objects.get(name)
    return o if (o and o.type == 'MESH') else None

def world_aabb_obj(o):
    mn = Vector((1e18,)*3); mx = Vector((-1e18,)*3); mw = o.matrix_world
    for c in o.bound_box:
        p = mw @ Vector(c)
        mn.x=min(mn.x,p.x); mn.y=min(mn.y,p.y); mn.z=min(mn.z,p.z)
        mx.x=max(mx.x,p.x); mx.y=max(mx.y,p.y); mx.z=max(mx.z,p.z)
    return mn, mx

def car_collection():
    for nm in ["x0_tyre_fl", "wheel_fl"]:
        o = bpy.data.objects.get(nm)
        if o and o.users_collection: return o.users_collection[0]
    return bpy.context.scene.collection

# ---------------- 検出(前回同様) ----------------
def number_region():
    mn = Vector((1e18,)*3); mx = Vector((-1e18,)*3); found=False; sx=0.0; sc=0
    nfaces=0; objs_hit=[]
    for o in bpy.data.objects:
        if o.type != 'MESH': continue
        nidx = set(i for i,s in enumerate(o.material_slots)
                   if s.material and NUMBER_TOKEN in s.material.name.lower())
        if not nidx: continue
        mw = o.matrix_world; oc=0
        for poly in o.data.polygons:
            if poly.material_index in nidx:
                found=True; nfaces+=1; oc+=1
                for vi in poly.vertices:
                    p = mw @ o.data.vertices[vi].co
                    mn.x=min(mn.x,p.x); mn.y=min(mn.y,p.y); mn.z=min(mn.z,p.z)
                    mx.x=max(mx.x,p.x); mx.y=max(mx.y,p.y); mx.z=max(mx.z,p.z)
                    sx += p.x; sc += 1
        if oc: objs_hit.append("%s(%d面)" % (o.name, oc))
    if not found:
        # 診断: なぜ番号マテリアルが見つからないか
        cand=set()
        for o in bpy.data.objects:
            if o.type!='MESH': continue
            for s in o.material_slots:
                if s.material and ("num" in s.material.name.lower()):
                    cand.add(s.material.name)
        log("  [診断] NUMBER_TOKEN='%s' を含む材質面が0。" % NUMBER_TOKEN)
        if cand:
            log("  [診断] 'num' を含む材質はある→トークン不一致かも: %s" % ", ".join(sorted(cand)))
            log("  [診断] → NUMBER_TOKEN を上記に合わせて変更してください。")
        else:
            log("  [診断] 'num' を含む材質自体が無い→この車は番号材質を使っていない可能性。")
        return None
    log("  [診断] 番号材質面 %d (%s)" % (nfaces, ", ".join(objs_hit)))
    return mn, mx, (sx/sc if sc else 0.0)

def fin_faces(topcanopy, nregion):
    nmn, nmx, xc = nregion; mw = topcanopy.matrix_world
    bm = bmesh.new(); bm.from_mesh(topcanopy.data); bm.faces.ensure_lookup_table()
    def wco(v): return mw @ v.co
    def is_thin(f): return all(abs(wco(v).x - xc) <= X_SLAB for v in f.verts)
    def fc(f):
        c = Vector((0,0,0))
        for v in f.verts: c += wco(v)
        return c/len(f.verts)
    gmin = Vector((min(nmn.x,xc)-FIN_GROW, nmn.y-FIN_GROW, nmn.z-FIN_GROW))
    gmax = Vector((max(nmx.x,xc)+FIN_GROW, nmx.y+FIN_GROW, nmx.z+FIN_GROW))
    def in_grow(c): return gmin.x<=c.x<=gmax.x and gmin.y<=c.y<=gmax.y and gmin.z<=c.z<=gmax.z
    smin = nmn - Vector((NUM_REGION_PAD,)*3); smax = nmx + Vector((NUM_REGION_PAD,)*3)
    def in_seedbox(c): return (smin.x<=c.x<=smax.x and smin.y<=c.y<=smax.y and smin.z<=c.z<=smax.z)
    def x_spread(f): return max(abs(wco(v).x - xc) for v in f.verts)  # 面のX方向の厚み(片側)
    thin_all = [f for f in bm.faces if is_thin(f)]
    inbox_all = [f for f in bm.faces if in_seedbox(fc(f))]
    seeds = [f for f in thin_all if in_seedbox(fc(f))]
    log("  [診断] 中心X(xc)=%.3f  X_SLAB=%.4f  シード箱X[%.2f..%.2f] Y[%.2f..%.2f] Z[%.2f..%.2f]"
        % (xc, X_SLAB, smin.x, smax.x, smin.y, smax.y, smin.z, smax.z))
    log("  [診断] 薄い面(全体)=%d / シード箱内の面=%d / シード(薄い∧箱内)=%d"
        % (len(thin_all), len(inbox_all), len(seeds)))
    if not seeds:
        if inbox_all:
            sp = sorted(x_spread(f) for f in inbox_all)
            need = sp[max(0, len(sp)//10)]  # 箱内面の下位10%が収まるX_SLAB目安
            log("  [診断] 箱内に面はあるが薄くない。箱内面のX厚み(片側)最小=%.4f 中央=%.4f"
                % (sp[0], sp[len(sp)//2]))
            log("  [診断] → X_SLAB を約 %.3f 以上に上げると拾えます(フィンが厚い/傾いている)。" % max(need, sp[0]*1.1))
        elif thin_all:
            txs=[fc(f).x for f in thin_all]; tys=[fc(f).y for f in thin_all]; tzs=[fc(f).z for f in thin_all]
            log("  [診断] 薄い面はあるがシード箱外。薄面の範囲 X[%.2f..%.2f] Y[%.2f..%.2f] Z[%.2f..%.2f]"
                % (min(txs),max(txs),min(tys),max(tys),min(tzs),max(tzs)))
            log("  [診断] → 番号領域がフィン上に無い/ズレ。NUM_REGION_PAD(現%.2f)を上げるか、"
                "番号(xc=%.3f)とフィン位置の関係を確認。" % (NUM_REGION_PAD, xc))
        else:
            log("  [診断] topcanopy_bm に『X方向に薄い面』が全く無い。")
            log("  [診断] → X_SLAB(現%.4f)が小さすぎるか、xc=%.3f がフィン中心とズレ。"
                "X_SLAB を上げて再試行。" % (X_SLAB, xc))
        bm.free(); return [], None
    visited = set(seeds); stack = list(seeds)
    while stack:
        f = stack.pop()
        for e in f.edges:
            for nf in e.link_faces:
                if nf not in visited and is_thin(nf) and in_grow(fc(nf)):
                    visited.add(nf); stack.append(nf)
    fidx = [f.index for f in visited]
    mn = Vector((1e18,)*3); mx = Vector((-1e18,)*3)
    for f in visited:
        for v in f.verts:
            p = wco(v)
            mn.x=min(mn.x,p.x); mn.y=min(mn.y,p.y); mn.z=min(mn.z,p.z)
            mx.x=max(mx.x,p.x); mx.y=max(mx.y,p.y); mx.z=max(mx.z,p.z)
    log("  [診断] シード%d→フラッドフィル後 %d 面 (FIN_GROW=%.2f)。"
        % (len(seeds), len(fidx), FIN_GROW))
    bm.free()
    return fidx, (mn, mx)

def thin_components(topcanopy, xc, xslab):
    """センターライン(|x-xc|<=xslab)の薄い面を連結成分に分け、各成分の面index/bbox/面数を返す。"""
    bm = bmesh.new(); bm.from_mesh(topcanopy.data); bm.faces.ensure_lookup_table()
    mw = topcanopy.matrix_world
    def wco(v): return mw @ v.co
    thin = set(f for f in bm.faces if all(abs(wco(v).x - xc) <= xslab for v in f.verts))
    comps = []; seen = set()
    for f0 in thin:
        if f0 in seen: continue
        stack = [f0]; comp = []; seen.add(f0)
        while stack:
            g = stack.pop(); comp.append(g)
            for e in g.edges:
                for nf in e.link_faces:
                    if nf in thin and nf not in seen:
                        seen.add(nf); stack.append(nf)
        mn = Vector((1e18,)*3); mx = Vector((-1e18,)*3)
        for f in comp:
            for v in f.verts:
                p = wco(v)
                mn.x=min(mn.x,p.x); mn.y=min(mn.y,p.y); mn.z=min(mn.z,p.z)
                mx.x=max(mx.x,p.x); mx.y=max(mx.y,p.y); mx.z=max(mx.z,p.z)
        comps.append(([f.index for f in comp], (mn, mx), len(comp)))
    total_thin = len(thin)
    bm.free()
    return comps, total_thin

def choose_fin(comps):
    """フィンらしい成分=一番背が高い(dz最大)縦板を選ぶ。"""
    best = None
    for fidx, (mn, mx), n in comps:
        if n < FIN_MIN_FACES: continue
        dz = mx.z - mn.z
        if best is None or dz > best[3]:
            best = (fidx, (mn, mx), n, dz, mx.y - mn.y, mx.x - mn.x)
    return best

def detect_fin_auto(topcanopy, xc):
    """X_SLAB を段階的に緩めて検出。届かなくても最良候補を採用(少しでも検出)。"""
    best_any = None
    for xslab in X_SLAB_CANDIDATES:
        comps, total_thin = thin_components(topcanopy, xc, xslab)
        cand = choose_fin(comps)
        if not cand:
            log("  [auto] X_SLAB=%.3f: 薄面%d 成分%d → 候補なし"
                % (xslab, total_thin, len(comps)))
            continue
        fidx, bb, n, dz, dy, dx = cand
        log("  [auto] X_SLAB=%.3f: 薄面%d 成分%d → 最良 面数=%d dz=%.3f dy=%.3f dx=%.3f"
            % (xslab, total_thin, len(comps), n, dz, dy, dx))
        if best_any is None or dz > best_any[3]:
            best_any = (fidx, bb, n, dz, dy, dx, xslab)
        if dz >= FIN_MIN_Z:
            log("  [auto] 採用: X_SLAB=%.3f dz=%.3f (>=FIN_MIN_Z=%.2f)" % (xslab, dz, FIN_MIN_Z))
            return fidx, bb, xslab
    if best_any:
        log("  [auto] FIN_MIN_Z(%.2f)未達だが最良候補を採用: X_SLAB=%.3f dz=%.3f (少しでも検出)"
            % (FIN_MIN_Z, best_any[6], best_any[3]))
        return best_any[0], best_any[1], best_any[6]
    return [], None, None

def separate_selected(obj, faceset, newname):
    if not faceset: return None
    b0 = set(o.name for o in bpy.data.objects)
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = obj; obj.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(obj.data); bm.faces.ensure_lookup_table()
    for f in bm.faces: f.select = False
    for fi in faceset:
        if fi < len(bm.faces): bm.faces[fi].select = True
    bmesh.update_edit_mesh(obj.data)
    bpy.ops.mesh.separate(type='SELECTED')
    bpy.ops.object.mode_set(mode='OBJECT')
    new = [o for o in bpy.data.objects if o.name not in b0]
    if not new: return None
    piece = new[0]; piece.name = newname; piece.data.name = newname
    return piece

def faces_in_bbox(obj, bbmin, bbmax, pad):
    mw = obj.matrix_world; mn = bbmin-Vector((pad,)*3); mx = bbmax+Vector((pad,)*3)
    idx = []
    for poly in obj.data.polygons:
        c = Vector((0,0,0))
        for vi in poly.vertices: c += mw @ obj.data.vertices[vi].co
        c /= len(poly.vertices)
        if mn.x<=c.x<=mx.x and mn.y<=c.y<=mx.y and mn.z<=c.z<=mx.z: idx.append(poly.index)
    return idx

def decal_sources():
    return [o for o in bpy.data.objects if o.type == 'MESH'
            and any(tok in o.name for tok in DECAL_TOKENS)]

def bake_decal_offset(decal):
    if decal.get("fw_offset"): return
    me = decal.data
    if me.users > 1: decal.data = me.copy(); me = decal.data
    for v in me.vertices: v.co = v.co + v.normal * DECAL_OFFSET
    me.update(); decal["fw_offset"] = DECAL_OFFSET

# ---------------- シーム検出(topcanopy近接) ----------------
def build_kd(obj):
    n = len(obj.data.vertices); kd = KDTree(n); mw = obj.matrix_world
    for i, v in enumerate(obj.data.vertices): kd.insert(mw @ v.co, i)
    kd.balance(); return kd

# ---------------- リグ ----------------
def reset_prior():
    o = bpy.data.objects.get(SF_ARM)
    if o:
        dat = o.data if (o.data and o.data.users == 1) else None
        bpy.data.objects.remove(o, do_unlink=True)
        if dat: bpy.data.armatures.remove(dat)
    for nm in ["sharkfin", "sharkfin_decal"]:
        a = mesh_obj(nm)
        if not a: continue
        for m in [m for m in a.modifiers if m.type == 'ARMATURE']: a.modifiers.remove(m)
        if a.parent and a.parent.type == 'ARMATURE':
            w = a.matrix_world.copy(); a.parent = None; a.matrix_world = w

def blend2(anchors, d):
    if d <= anchors[0][1]: return (anchors[0][0], 1.0), None
    if d >= anchors[-1][1]: return (anchors[-1][0], 1.0), None
    for i in range(len(anchors)-1):
        d0=anchors[i][1]; d1=anchors[i+1][1]
        if d0 <= d <= d1:
            f=(d-d0)/(d1-d0) if d1>d0 else 0.0
            return (anchors[i][0],1.0-f),(anchors[i+1][0],f)
    return (anchors[-1][0],1.0), None

def rot_world(bone, axis_world, ang, frame):
    axis_local = bone.bone.matrix_local.to_3x3().inverted() @ axis_world
    if axis_local.length < 1e-9: return
    bone.rotation_quaternion = Quaternion(axis_local.normalized(), ang)
    bone.keyframe_insert("rotation_quaternion", frame=frame)

def build_rig(fin, decal, tc):
    mn, mx = world_aabb_obj(fin)
    cx = (mn.x+mx.x)*0.5; cz = (mn.z+mx.z)*0.5
    # 前後判定(リアウィングに近い側=後)
    rw = mesh_obj(REARWING); rear_ref = mx.y
    if rw:
        rmn, rmx = world_aabb_obj(rw); rear_ref = (rmn.y+rmx.y)*0.5
    if abs(mn.y-rear_ref) < abs(mx.y-rear_ref):
        rear_y, front_y = mn.y, mx.y
    else:
        rear_y, front_y = mx.y, mn.y
    fin_len = abs(rear_y-front_y) or 1e-4
    log("前端Y=%.3f 後端Y=%.3f (後=リア側)" % (front_y, rear_y))

    arm = bpy.data.armatures.new(SF_ARM)
    arm_obj = bpy.data.objects.new(SF_ARM, arm)
    car_collection().objects.link(arm_obj); arm_obj.location = Vector((0,0,0))
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='EDIT')
    eb = arm.edit_bones
    fracs = [i/SF_BONES for i in range(SF_BONES+1)]   # 前→後
    prev = None
    for i in range(SF_BONES):
        b = eb.new(SF_BONES_NAMES[i])
        y0 = front_y + (rear_y-front_y)*fracs[i]
        y1 = front_y + (rear_y-front_y)*fracs[i+1]
        b.head = Vector((cx, y0, cz)); b.tail = Vector((cx, y1, cz))
        if prev: b.parent=prev; b.use_connect=True
        prev=b
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.view_layer.update()

    anchors=[]
    for nm in SF_BONES_NAMES:
        bb = arm.bones.get(nm)
        mid = arm_obj.matrix_world @ ((bb.head_local+bb.tail_local)*0.5)
        anchors.append((nm, abs(mid.y - front_y)))
    anchors.sort(key=lambda x:x[1])
    root = SF_BONES_NAMES[0]

    kd = build_kd(tc)
    def skin(obj):
        for nm in SF_BONES_NAMES:
            if not obj.vertex_groups.get(nm): obj.vertex_groups.new(name=nm)
        gi = {vg.name: vg.index for vg in obj.vertex_groups}
        mw = obj.matrix_world; nseam = 0
        for v in obj.data.vertices:
            p = mw @ v.co
            # シーム固定: topcanopy近接 or 前方ロック帯
            _, _, dist = kd.find(p)
            yfrac = abs(p.y - front_y)/fin_len
            if (dist is not None and dist <= SEAM_EPS) or yfrac <= FRONT_LOCK_RATIO:
                obj.vertex_groups[gi[root]].add([v.index], 1.0, 'REPLACE'); nseam += 1; continue
            (n0,w0), second = blend2(anchors, abs(p.y - front_y))
            obj.vertex_groups[gi[n0]].add([v.index], w0, 'REPLACE')
            if second: obj.vertex_groups[gi[second[0]]].add([v.index], second[1], 'REPLACE')
        if not any(m.type=='ARMATURE' for m in obj.modifiers):
            m=obj.modifiers.new("Armature",'ARMATURE'); m.object=arm_obj
        else:
            for m in obj.modifiers:
                if m.type=='ARMATURE': m.object=arm_obj
        obj.parent = arm_obj
        obj.matrix_parent_inverse = arm_obj.matrix_world.inverted()
        log("  %s: シーム/前方固定 %d 頂点" % (obj.name, nseam))

    skin(fin)
    if decal:
        skin(decal)
        if DECAL_FLOATING: bake_decal_offset(decal)

    # キーフレーム(プレビュー用/片側): ワールドZ軸 yaw, 前=0→後ほど大
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='POSE')
    pb = arm_obj.pose.bones
    bpy.context.scene.frame_set(1)
    for n in SF_BONES_NAMES:
        b = pb.get(n)
        if not b: continue
        b.rotation_mode='QUATERNION'; b.rotation_quaternion=(1,0,0,0)
        b.keyframe_insert("rotation_quaternion", frame=1)
    bpy.context.scene.frame_set(ANIM_END_FRAME)
    for i, n in enumerate(SF_BONES_NAMES):
        b = pb.get(n)
        if not b: continue
        ang = radians(i*SF_BEND_DEG_PER_STEP*SF_BEND_SCALE)*SF_BEND_SIGN
        rot_world(b, Vector((0,0,1)), ang, ANIM_END_FRAME)   # Z軸=左右yaw
    bpy.ops.object.mode_set(mode='OBJECT'); bpy.context.scene.frame_set(1)

def wobble_text():
    lines=[]; wi=0
    for nm in SF_BONES_NAMES[1:]:   # 根元(前)は固定→除外
        lines.append(
            "[WOBBLY_BIT_%d]\nNAME = %s\nCONNECTED_TO = 0, 0, 0\nMAX_RANGE = %s\n"
            "DAMPENING_LAG = %s\nG_GAIN = %s\nGRAVITY_GAIN = %s\nOFFSET_GAIN = %s\n"
            "STIFF_AXIS = %s\nSTIFF_AXIS_STIFFNESS = %s\nG_FILTER = %s\n"
            "DEFAULT_GRAVITY_INCLUDED_ALREADY = 1\n"
            % (wi, nm, WOBBLE["MAX_RANGE"], WOBBLE["DAMPENING_LAG"], WOBBLE["G_GAIN"],
               WOBBLE["GRAVITY_GAIN"], WOBBLE["OFFSET_GAIN"], WOBBLE["STIFF_AXIS"],
               WOBBLE["STIFF"], WOBBLE["G_FILTER"])); wi+=1
    return ("; --- ext_config.ini: 左右対称の揺れ(アンテナと同じ物理揺れ) ---\n"
            "; ksanim(速度で片側)ではなくこちらを使うと静止=中心で左右対称に揺れます\n"
            + "\n".join(lines))

def ini_text():
    aero = ("[WING_%d]\nNAME=sharkfin_flex\nCHORD=1\nSPAN=1\nPOSITION=0,0,0\n"
            "LUT_AOA_CL=Neutral.lut\nLUT_GH_CL=Neutral.lut\nCL_GAIN=0\n"
            "LUT_AOA_CD=Neutral.lut\nLUT_GH_CD=Neutral.lut\nCD_GAIN=0\nANGLE=0\n"
            "ZONE_FRONT_CL=0\nZONE_FRONT_CD=0\nZONE_REAR_CL=0\nZONE_REAR_CD=0\n"
            "ZONE_LEFT_CL=0\nZONE_LEFT_CD=0\nZONE_RIGHT_CL=0\nZONE_RIGHT_CD=0\nYAW_CL_GAIN=0\n\n"
            "[DYNAMIC_CONTROLLER_%d]\nWING=%d\nCOMBINATOR=ADD\nINPUT=SPEED_KMH\n"
            "LUT=sharkfin_flex_anim.lut\nFILTER=0.9\nUP_LIMIT=100\nDOWN_LIMIT=0\n"
            % (SF_WING_INDEX, SF_WING_INDEX, SF_WING_INDEX))
    wa = "[ANIMATION_%d]\nWING=%d\nFILE=%s\nMIN=0\nMAX=100\n" % (SF_WING_INDEX, SF_WING_INDEX, SF_KSANIM)
    return ("; ===== (任意)速度で片側に曲げたい場合の aero/wing_animation =====\n%s\n%s\n\n%s"
            % (aero, wa, wobble_text()))

def _dump():
    print("\n##### COPY (レポート) #####"); print("\n".join(REPORT))

def main():
    log("============ 操作10 シャークフィン rev3 (DETECT_ONLY=%s) ============" % DETECT_ONLY)
    tc = mesh_obj(TOPCANOPY)
    if not tc: log("[ERROR] %s 無し" % TOPCANOPY); _dump(); return
    fin = mesh_obj("sharkfin"); decal = mesh_obj("sharkfin_decal")
    if not fin:
        if AUTO_DETECT:
            # 番号に依存しない自動検出(センターラインの薄い縦板の最良成分)
            nreg = number_region()   # xc の参考 + 診断用(必須ではない)
            xc = FIN_CENTER_X
            if nreg and abs(nreg[2] - FIN_CENTER_X) < 0.1:
                xc = nreg[2]
            log("自動検出: xc=%.3f から X_SLAB を段階探索" % xc)
            fidx, fbb, used_slab = detect_fin_auto(tc, xc)
            if not fidx:
                log("シャークフィン未検出(自動)。センターラインに薄い縦板が見つかりません。"
                    "→ この車はフィン無し、または FIN_CENTER_X/X_SLAB_CANDIDATES を確認。")
                _dump(); return
            fmn, fmx = fbb; dz=fmx.z-fmn.z; dx=fmx.x-fmn.x; dy=fmx.y-fmn.y
            log("フィン確定(自動): 面数=%d X=%.3f Y=%.3f Z=%.3f (X_SLAB=%.3f)"
                % (len(fidx), dx, dy, dz, used_slab))
        else:
            nreg = number_region()
            if not nreg:
                log("番号マテリアル無し→シャークフィン無しと判断。終了。"); _dump(); return
            nmn, nmx, xc = nreg
            log("番号領域: Y[%.3f..%.3f] Z[%.3f..%.3f] 中心X=%.3f" % (nmn.y,nmx.y,nmn.z,nmx.z,xc))
            fidx, fbb = fin_faces(tc, nreg)
            if not fidx:
                log("薄いフィン面(シード)なし→シャークフィン未検出。終了。上の[診断]の指示に従い "
                    "X_SLAB / NUM_REGION_PAD を調整して再試行してください。"); _dump(); return
            fmn, fmx = fbb; dz=fmx.z-fmn.z; dx=fmx.x-fmn.x; dy=fmx.y-fmn.y
            log("フィン候補: 面数=%d X=%.3f Y=%.3f Z=%.3f" % (len(fidx),dx,dy,dz))
            if dz < FIN_MIN_Z:
                log("Z高さ不足→未検出。dz=%.3f < FIN_MIN_Z=%.3f。" % (dz, FIN_MIN_Z))
                log("  [診断] → FIN_MIN_Z を下げるか FIN_GROW(現%.2f)を上げる。" % FIN_GROW)
                _dump(); return
        dsel = []
        for d in decal_sources():
            di = faces_in_bbox(d, fmn, fmx, DECAL_PAD)
            if di: dsel.append((d,di)); log("  デカール候補 %s: %d 面" % (d.name, len(di)))
        if DETECT_ONLY: log("DETECT_ONLY: 分離せず終了。"); _dump(); return
        fin = separate_selected(tc, fidx, "sharkfin")
        if not fin: log("[ERROR] 分離失敗"); _dump(); return
        for k,(d,di) in enumerate(dsel):
            nm = "sharkfin_decal" if k==0 else "sharkfin_decal_%d" % (k+1)
            separate_selected(d, di, nm)
        decal = mesh_obj("sharkfin_decal")
        log("分離: sharkfin (+デカール %d)" % len(dsel))

    if RESET: reset_prior()
    build_rig(fin, decal, tc)
    log("リグ完了: %s (Y鎖/Z軸yaw, シーム固定, 後方のみ左右)" % SF_ARM)
    txt = ini_text()
    blend = bpy.data.filepath
    if blend:
        path = os.path.join(os.path.dirname(blend), OUT_TXT)
        try:
            with open(path, "w", encoding="utf-8") as f: f.write(txt + "\n")
            log("iniテキスト: %s" % path)
        except Exception as e:
            log("[WARN] 書き出し失敗: %s" % e)
    log("確認: 再生でリア端だけ左右に振れ、境界/前方/中央が動かないか")
    log("      左右対称の揺れは WOBBLY_BIT を ext_config に追記(番号衝突注意)")
    _dump()
    print("\n##### ↓ ini/wobbly テキスト ↓ #####"); print(txt); print("##### ここまで #####")

if __name__ == "__main__":
    main()
