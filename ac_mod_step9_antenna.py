# -*- coding: utf-8 -*-
"""
操作9: アンテナ検出＋しなりアニメ (rev2)
================================================================================
rev2の変更:
 【問題1: しならない】WOBBLY_BITだけだとBlender上で動かない(CSP実機のみ)。
   → 翼と同じ "キーフレームのしなり"(根元固定・上部が後方へ曲がる)を追加。
     Blenderで再生でき ksanim 化も可能。WOBBLY_BITテキストも併せて出力。
 【問題2: キャップ】アンテナ本体の上部数cm以内にある _detail 島(=キャップ)を
   safetycell から検出・分離して対応アンテナへ join。
   既存 antenna_X があれば「キャップ回収＋リグ」を実行。無ければ新規検出。

安全: DETECT_ONLY=True で検出レポートのみ(分離しない)。確認後 False で実行。
"""
import bpy, os, bmesh
from mathutils import Vector, Quaternion
from math import radians

# ============================== CONFIG ==============================
DETECT_ONLY = False         # True=検出レポートのみ / False=分離+リグ実行

SAFETYCELL = "safetycell"
HAL = "hal"
DETAIL_SUFFIX = "_detail"

# 検出しきい値(ワールド単位)
ANT_MIN_HEIGHT = 0.04
ANT_THIN_MAX   = 0.04
ANT_RATIO_MIN  = 3.0
REGION_PAD     = 0.05
MAX_ANTENNAS   = 4
EXPECT_ANTENNAS = 2         # 目安の本数(多くの車は2本)。ログ/次点表示に使用
ANT_ABS_RATIO  = 3.0        # 採用の絶対ratio下限
ANT_REL_FRAC   = 0.15       # 最上位ratio×この割合 以上も採用(2本目=似た高ratioを拾う)
# キャップ(アンテナ上部の少し太い部分)
CAP_DIST = 0.05             # アンテナ頂点から上へこの距離以内
CAP_XY   = 0.05             # XY中心がこの距離以内

# 段階探索(できる限り探す。無い時はレベルを使い切って諦める)
AUTO_ESCALATE = True
# (MIN_HEIGHT, THIN_MAX, RATIO_MIN, REGION_PAD) を厳→緩へ。最初に本体が見つかったレベルを採用。
ANT_LEVELS = [
    (0.040, 0.040, 3.0, 0.05),
    (0.030, 0.060, 2.5, 0.10),
    (0.020, 0.080, 2.0, 0.20),
    (0.015, 0.100, 1.6, 0.35),
    (0.010, 0.130, 1.4, 0.50),
]
# (CAP_DIST, CAP_XY) を厳→緩へ。各アンテナごとに見つかるまで緩める。
CAP_LEVELS = [
    (0.05, 0.05),
    (0.08, 0.07),
    (0.12, 0.10),
    (0.18, 0.14),
]

# リグ / しなり(キーフレーム)
ANT_BONES = 3               # 根元含むボーン数。根元固定, 上が曲がる
ANIM_END_FRAME = 100
ANT_BEND_SCALE = 0.6        # しなり倍率
ANT_BEND_DEG_PER_STEP = 7.0 # 上へ1節ごとの基準角(度)
ANT_BEND_SIGN = +1.0        # 前後の向き(逆なら反転)
ANT_ARM = "antenna_armature"
# 実機の風ジッター(WOBBLY_BIT)
WOBBLE = dict(MAX_RANGE=0.18, DAMPENING_LAG=0.90, G_GAIN=0.9, GRAVITY_GAIN=1.0,
              OFFSET_GAIN=2000, STIFF_AXIS="0, 0, 1", STIFF="0.08", G_FILTER="0.0")
OUT_TXT = "_antenna_ext_config.txt"
RESET = True
# ===================================================================

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

def xy_center(mn, mx): return Vector(((mn.x+mx.x)*0.5, (mn.y+mx.y)*0.5))

def car_collection():
    for nm in ["x0_tyre_fl", "wheel_fl"]:
        o = bpy.data.objects.get(nm)
        if o and o.users_collection: return o.users_collection[0]
    return bpy.context.scene.collection

def existing_antennas():
    return sorted([o for o in bpy.data.objects
                   if o.type == 'MESH' and o.name.startswith("antenna_")],
                  key=lambda o: o.name)

def is_tall_thin(o):
    mn, mx = world_aabb_obj(o)
    dx = mx.x-mn.x; dy = mx.y-mn.y; dz = mx.z-mn.z
    return (dz >= ANT_MIN_HEIGHT) and (dz/max(dx,dy,1e-6) >= ANT_RATIO_MIN) \
           and dx < ANT_THIN_MAX and dy < ANT_THIN_MAX

# ---------------- detail島の検出 ----------------
def detail_slot_indices(obj):
    return set(i for i, s in enumerate(obj.material_slots)
               if s.material and DETAIL_SUFFIX in s.material.name.lower())

def detail_islands(obj):
    didx = detail_slot_indices(obj)
    if not didx: return []
    bm = bmesh.new(); bm.from_mesh(obj.data); bm.faces.ensure_lookup_table()
    dset = set(f for f in bm.faces if f.material_index in didx)
    visited = set(); islands = []; mw = obj.matrix_world
    for f0 in dset:
        if f0 in visited: continue
        stack = [f0]; comp = []
        while stack:
            g = stack.pop()
            if g in visited: continue
            visited.add(g); comp.append(g)
            for e in g.edges:
                for gf in e.link_faces:
                    if gf in dset and gf not in visited: stack.append(gf)
        mn = Vector((1e18,)*3); mx = Vector((-1e18,)*3); fidx = []
        for f in comp:
            fidx.append(f.index)
            for v in f.verts:
                p = mw @ v.co
                mn.x=min(mn.x,p.x); mn.y=min(mn.y,p.y); mn.z=min(mn.z,p.z)
                mx.x=max(mx.x,p.x); mx.y=max(mx.y,p.y); mx.z=max(mx.z,p.z)
        islands.append((fidx, mn, mx))
    bm.free()
    return islands

def classify_antenna(mn, mx, sc_front, hal_front):
    dx=mx.x-mn.x; dy=mx.y-mn.y; dz=mx.z-mn.z; cy=(mn.y+mx.y)*0.5
    thin=(dx<ANT_THIN_MAX) and (dy<ANT_THIN_MAX); tall=dz>=ANT_MIN_HEIGHT
    ratio=dz/max(dx,dy,1e-6)
    region=(sc_front-REGION_PAD)<=cy<=(hal_front+REGION_PAD)
    return (thin and tall and ratio>=ANT_RATIO_MIN and region), \
           dict(dx=dx,dy=dy,dz=dz,cy=cy,ratio=ratio,region=region)

def classify_at(mn, mx, sc_front, hal_front, H, THIN, RATIO, PAD):
    dx=mx.x-mn.x; dy=mx.y-mn.y; dz=mx.z-mn.z; cy=(mn.y+mx.y)*0.5
    ratio=dz/max(dx,dy,1e-6)
    thin=(dx<THIN) and (dy<THIN); tall=dz>=H
    region=(sc_front-PAD)<=cy<=(hal_front+PAD)
    return (thin and tall and ratio>=RATIO and region), \
           dict(dx=dx,dy=dy,dz=dz,cy=cy,ratio=ratio,region=region)

def is_cap_at(mn, mx, amn, amx, CD, CXY):
    above = (amx.z - 0.01) <= mn.z <= (amx.z + CD)
    close = (xy_center(mn,mx) - xy_center(amn,amx)).length < CXY
    return above and close

def is_cap_of(mn, mx, amn, amx):
    """島(mn,mx)が アンテナ(amn,amx)の上部キャップか。"""
    above = (amx.z - 0.01) <= mn.z <= (amx.z + CAP_DIST)
    close = (xy_center(mn,mx) - xy_center(amn,amx)).length < CAP_XY
    return above and close

# ---------------- 分離ユーティリティ ----------------
def separate_faces_to_pieces(sc, faceset):
    if not faceset: return []
    b0 = set(o.name for o in bpy.data.objects)
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = sc; sc.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(sc.data); bm.faces.ensure_lookup_table()
    for f in bm.faces: f.select = False
    for fi in faceset:
        if fi < len(bm.faces): bm.faces[fi].select = True
    bmesh.update_edit_mesh(sc.data)
    bpy.ops.mesh.separate(type='SELECTED')
    bpy.ops.object.mode_set(mode='OBJECT')
    holders = [o for o in bpy.data.objects if o.name not in b0]
    pieces = []
    for h in holders:
        b1 = set(o.name for o in bpy.data.objects)
        bpy.ops.object.select_all(action='DESELECT')
        bpy.context.view_layer.objects.active = h; h.select_set(True)
        bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.separate(type='LOOSE'); bpy.ops.object.mode_set(mode='OBJECT')
        pieces.append(h)
        pieces += [o for o in bpy.data.objects if o.name not in b1]
    return pieces

def join_caps(cap_names, antenna_names):
    """名前ベースで安全に join。各joinの直前にオブジェクトを取り直す。"""
    ants = []
    for an in antenna_names:
        a = bpy.data.objects.get(an)
        if a: ants.append((an, world_aabb_obj(a)))
    pairs = []
    for cn in cap_names:
        c = bpy.data.objects.get(cn)
        if not c: continue
        cmn, cmx = world_aabb_obj(c); cxy = xy_center(cmn, cmx)
        best = None; bd = 1e18
        for an, (amn, amx) in ants:
            if cmn.z < amx.z - 0.02: continue          # cがaの上でない
            score = (cxy - xy_center(amn, amx)).length + abs(cmn.z - amx.z)
            if score < bd: bd = score; best = an
        if best: pairs.append((cn, best))
    for cn, an in pairs:
        c = bpy.data.objects.get(cn); a = bpy.data.objects.get(an)
        if not c or not a: continue
        bpy.ops.object.select_all(action='DESELECT')
        c.select_set(True); a.select_set(True)
        bpy.context.view_layer.objects.active = a
        try:
            bpy.ops.object.join()
            log("  キャップ %s → %s に統合" % (cn, an))
        except Exception as e:
            log("  [join失敗] %s→%s: %s" % (cn, an, e))

def antenna_material(sc):
    for s in sc.material_slots:
        if s.material and DETAIL_SUFFIX in s.material.name.lower():
            nm = s.material.name + "_antenna"
            m = bpy.data.materials.get(nm) or s.material.copy()
            m.name = nm; return m
    return None

def assign_material(ant, mat):
    if not mat: return
    ant.data.materials.clear(); ant.data.materials.append(mat)
    for p in ant.data.polygons: p.material_index = 0

# ---------------- 検出フロー ----------------
def detect_fresh(sc):
    islands = detail_islands(sc)   # [(fidx, mn, mx), ...]
    if not islands: log("[ERROR] detailマテリアル面なし"); return set(), []
    scmn, scmx = world_aabb_obj(sc); sc_front = scmn.y
    hal = mesh_obj(HAL)
    hal_front = world_aabb_obj(hal)[0].y if hal else scmn.y + (scmx.y-scmn.y)*0.4
    log("検出領域Y: [%.3f .. %.3f]  detail島=%d" % (sc_front, hal_front, len(islands)))

    # --- 本体: 最も緩いレベルで候補を集め、ratio上位を採用(2本目も拾う) ---
    H, THIN, RATIO, PAD = (ANT_LEVELS[-1] if AUTO_ESCALATE else ANT_LEVELS[0])
    metr = []   # (k, info, ok)
    for k, (fidx, mn, mx) in enumerate(islands):
        ok, info = classify_at(mn, mx, sc_front, hal_front, H, THIN, RATIO, PAD)
        metr.append((k, info, ok))
    # 診断: 全detail島を ratio 上位8で表示(★=候補)
    log("  [診断] detail島 ratio上位8 (★=本体候補, 緩レベル H>=%.3f thin<%.2f 比>=%.1f pad%.2f):"
        % (H, THIN, RATIO, PAD))
    for k, info, ok in sorted(metr, key=lambda t: t[1]["ratio"], reverse=True)[:8]:
        log("    %s dz=%.3f dx=%.3f dy=%.3f 比=%.1f cy=%.3f 領域=%s"
            % ("★" if ok else "  ", info["dz"], info["dx"], info["dy"],
               info["ratio"], info["cy"], info["region"]))

    cands = sorted([(k, info) for k, info, ok in metr if ok],
                   key=lambda t: t[1]["ratio"], reverse=True)
    if not cands:
        log("  → アンテナ本体候補0。上の寸法を見て ANT_LEVELS 最終行(H/thin/比/pad)を調整。無い可能性も。")
        return set(), []

    # ratio が最上位に近い島を採用(2本のアンテナは似た高ratio。低ratioの雑島は除外)
    top = cands[0][1]["ratio"]
    thr = max(ANT_ABS_RATIO, top * ANT_REL_FRAC)
    ant_idx = [c for c in cands if c[1]["ratio"] >= thr][:MAX_ANTENNAS]
    used = (H, THIN, RATIO, PAD)
    log("  採用しきい ratio>=%.1f → 本体%d本 (目安%d本, 候補全%d)"
        % (thr, len(ant_idx), EXPECT_ANTENNAS, len(cands)))
    if len(ant_idx) < EXPECT_ANTENNAS and len(cands) > len(ant_idx):
        nxt = cands[len(ant_idx)][1]
        log("  [診断] 目安より少ない。次点島: 比=%.1f dz=%.3f cy=%.3f "
            "(採用に含めたいなら ANT_REL_FRAC を下げる/ANT_ABS_RATIO を下げる)"
            % (nxt["ratio"], nxt["dz"], nxt["cy"]))

    ant_islands = [(islands[k][0], islands[k][1], islands[k][2]) for k, _ in ant_idx]
    faces = set(); used_k = set(k for k, _ in ant_idx)
    log("アンテナ本体 %d 本:" % len(ant_idx))
    for (fidx, mn, mx), (k, info) in zip(ant_islands, ant_idx):
        log("  ★ 高さZ=%.3f X=%.3f Y=%.3f 比=%.1f cy=%.3f" %
            (info["dz"], info["dx"], info["dy"], info["ratio"], info["cy"]))
        faces.update(fidx)

    # --- キャップ: 各本体ごとに CAP_LEVELS を緩めながら探索(割当済みは除外) ---
    ncap = 0; assigned = set()
    cap_levels = CAP_LEVELS if AUTO_ESCALATE else CAP_LEVELS[:1]
    for (afidx, amn, amx) in ant_islands:
        found = False; nearest = None
        for (CD, CXY) in cap_levels:
            for k, (fidx, mn, mx) in enumerate(islands):
                if k in used_k or k in assigned: continue
                gap = mn.z - amx.z
                dxy = (xy_center(mn,mx) - xy_center(amn,amx)).length
                if nearest is None or (gap >= -0.01 and gap < nearest[0]):
                    nearest = (gap, dxy, mx.z-mn.z)
                if is_cap_at(mn, mx, amn, amx, CD, CXY):
                    faces.update(fidx); assigned.add(k); ncap += 1; found = True
                    log("  ＋キャップ島(高さ%.3f, 上ギャップ%.3f XY%.3f ≤ CAP_DIST%.2f/CAP_XY%.2f)"
                        % (mx.z-mn.z, gap, dxy, CD, CXY))
                    break
            if found: break
        if not found and nearest is not None:
            log("  [診断] 本体(頂点Z=%.3f)のキャップ未検出。最寄り島: 上ギャップ%.3f XY%.3f 高さ%.3f"
                % (amx.z, nearest[0], nearest[1], nearest[2]))
            log("    → CAP_LEVELS を上ギャップ%.3f/XY%.3f 以上に緩めれば拾えます(誤検出注意)。"
                % (max(nearest[0],0.0), nearest[1]))

    log("→ アンテナ%d本 + キャップ%d個 (本体レベル H>=%.3f)" % (len(ant_idx), ncap, used[0]))
    ant_boxes = [(mn, mx) for _, mn, mx in ant_islands]
    return faces, ant_boxes

def detect_caps_for(sc, antennas):
    islands = detail_islands(sc)
    faces = set(); n = 0; assigned = set()
    cap_levels = CAP_LEVELS if AUTO_ESCALATE else CAP_LEVELS[:1]
    for a in antennas:
        amn, amx = world_aabb_obj(a); found=False; nearest=None
        for (CD, CXY) in cap_levels:
            for k, (fidx, mn, mx) in enumerate(islands):
                if k in assigned: continue
                gap = mn.z - amx.z
                dxy = (xy_center(mn,mx) - xy_center(amn,amx)).length
                if nearest is None or (gap >= -0.01 and gap < nearest[0]):
                    nearest = (gap, dxy, mx.z-mn.z)
                if is_cap_at(mn, mx, amn, amx, CD, CXY):
                    faces.update(fidx); assigned.add(k); n += 1; found=True
                    log("  ＋既存 %s 用キャップ島(高さ%.3f, CAP_DIST≤%.2f/XY≤%.2f)"
                        % (a.name, mx.z-mn.z, CD, CXY)); break
            if found: break
        if not found and nearest is not None:
            log("  [診断] %s のキャップ未検出。最寄り: 上ギャップ%.3f XY%.3f 高さ%.3f"
                % (a.name, nearest[0], nearest[1], nearest[2]))
    log("既存アンテナ用キャップ: %d 個" % n)
    return faces

# ---------------- リグ + キーフレーム ----------------
def reset_prior():
    o = bpy.data.objects.get(ANT_ARM)
    if o:
        dat = o.data if (o.data and o.data.users == 1) else None
        bpy.data.objects.remove(o, do_unlink=True)
        if dat: bpy.data.armatures.remove(dat)
    for a in existing_antennas():
        for m in [m for m in a.modifiers if m.type == 'ARMATURE']:
            a.modifiers.remove(m)
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

def build_rig(ants):
    arm = bpy.data.armatures.new(ANT_ARM)
    arm_obj = bpy.data.objects.new(ANT_ARM, arm)
    car_collection().objects.link(arm_obj); arm_obj.location = Vector((0,0,0))
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='EDIT')
    eb = arm.edit_bones
    fracs = [i/ANT_BONES for i in range(ANT_BONES+1)]
    per = {}
    for ai, a in enumerate(ants, start=1):
        mn, mx = world_aabb_obj(a)
        cx=(mn.x+mx.x)*0.5; cy=(mn.y+mx.y)*0.5; z0=mn.z; h=max(mx.z-mn.z,1e-4)
        names=[]; prev=None
        for i in range(ANT_BONES):
            b = eb.new("Ant%d_%d" % (ai, i))
            b.head = Vector((cx, cy, z0+fracs[i]  *h))
            b.tail = Vector((cx, cy, z0+fracs[i+1]*h))
            if prev: b.parent=prev; b.use_connect=True
            prev=b; names.append(b.name)
        per[a.name] = (names, z0)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.view_layer.update()

    # スキン(Zで位置ウェイト)
    for a in ants:
        names, z0 = per[a.name]
        anchors=[]
        for nm in names:
            bb = arm.bones.get(nm)
            mid = arm_obj.matrix_world @ ((bb.head_local+bb.tail_local)*0.5)
            anchors.append((nm, abs(mid.z - z0)))
        anchors.sort(key=lambda x:x[1])
        for nm in names:
            if not a.vertex_groups.get(nm): a.vertex_groups.new(name=nm)
        gi = {vg.name: vg.index for vg in a.vertex_groups}
        mw = a.matrix_world
        for v in a.data.vertices:
            z=(mw@v.co).z
            (n0,w0),second = blend2(anchors, abs(z-z0))
            a.vertex_groups[gi[n0]].add([v.index], w0, 'REPLACE')
            if second: a.vertex_groups[gi[second[0]]].add([v.index], second[1], 'REPLACE')
        if not any(m.type=='ARMATURE' for m in a.modifiers):
            m=a.modifiers.new("Armature",'ARMATURE'); m.object=arm_obj
        else:
            for m in a.modifiers:
                if m.type=='ARMATURE': m.object=arm_obj
        a.parent = arm_obj
        a.matrix_parent_inverse = arm_obj.matrix_world.inverted()

    # キーフレーム(根元固定・上が後方へ曲がる, ワールドX軸)
    bpy.context.view_layer.objects.active = arm_obj
    bpy.ops.object.mode_set(mode='POSE')
    pb = arm_obj.pose.bones
    allnames = [n for names,_ in per.values() for n in names]
    bpy.context.scene.frame_set(1)
    for n in allnames:
        b = pb.get(n)
        if not b: continue
        b.rotation_mode='QUATERNION'; b.rotation_quaternion=(1,0,0,0)
        b.keyframe_insert("rotation_quaternion", frame=1)
    bpy.context.scene.frame_set(ANIM_END_FRAME)
    for names, _ in per.values():
        for i, n in enumerate(names):
            b = pb.get(n)
            if not b: continue
            ang = radians(i*ANT_BEND_DEG_PER_STEP*ANT_BEND_SCALE)*ANT_BEND_SIGN
            rot_world(b, Vector((1,0,0)), ang, ANIM_END_FRAME)
    bpy.ops.object.mode_set(mode='OBJECT'); bpy.context.scene.frame_set(1)
    return per

def ext_config_text(per):
    lines=[]; wi=0
    for names, _ in per.values():
        for nm in names[1:]:   # 根元は固定→除外
            lines.append(
                "[WOBBLY_BIT_%d]\nNAME = %s\nCONNECTED_TO = 0, 0, 0\nMAX_RANGE = %s\n"
                "DAMPENING_LAG = %s\nG_GAIN = %s\nGRAVITY_GAIN = %s\nOFFSET_GAIN = %s\n"
                "STIFF_AXIS = %s\nSTIFF_AXIS_STIFFNESS = %s\nG_FILTER = %s\n"
                "DEFAULT_GRAVITY_INCLUDED_ALREADY = 1\n"
                % (wi, nm, WOBBLE["MAX_RANGE"], WOBBLE["DAMPENING_LAG"], WOBBLE["G_GAIN"],
                   WOBBLE["GRAVITY_GAIN"], WOBBLE["OFFSET_GAIN"], WOBBLE["STIFF_AXIS"],
                   WOBBLE["STIFF"], WOBBLE["G_FILTER"])); wi+=1
    return ("; アンテナ: Blenderのキーフレームで見た目確認/ksanim化できます。\n"
            "; 実機で風ジッターも欲しい場合のみ下記 WOBBLY_BIT を ext_config へ(番号衝突注意)。\n"
            + "\n".join(lines))

def _dump():
    print("\n##### COPY (レポート) #####"); print("\n".join(REPORT))

def main():
    log("============ 操作9 アンテナ rev2 (DETECT_ONLY=%s) ============" % DETECT_ONLY)
    sc = mesh_obj(SAFETYCELL)
    ants = existing_antennas()

    if ants:
        log("既存アンテナ: %s → キャップ回収を試行" % ", ".join(a.name for a in ants))
        if sc:
            capf = detect_caps_for(sc, ants)
            if DETECT_ONLY:
                log("DETECT_ONLY: 回収せず終了。"); _dump(); return
            if capf:
                pieces = separate_faces_to_pieces(sc, capf)
                join_caps([p.name for p in pieces], [a.name for a in ants])
                mat = antenna_material(sc)
                for a in ants: assign_material(a, mat)
    else:
        if not sc: log("[ERROR] safetycell無し"); _dump(); return
        faces, ant_boxes = detect_fresh(sc)
        if not faces: log("候補なし。しきい値調整して再実行。"); _dump(); return
        if DETECT_ONLY:
            log("DETECT_ONLY: 分離せず終了。★が想定通りなら False で再実行。"); _dump(); return
        pieces = separate_faces_to_pieces(sc, faces)
        # 検出時の本体bboxと一致するピースを本体と判定(サイズ非依存で2本目も拾う)
        def piece_is_antenna(p):
            pmn, pmx = world_aabb_obj(p); pc = xy_center(pmn, pmx); pz = pmx.z - pmn.z
            for (amn, amx) in ant_boxes:
                az = amx.z - amn.z
                if ((pc - xy_center(amn, amx)).length < 0.02
                        and abs(pmn.z - amn.z) < 0.02
                        and abs(pz - az) < max(0.012, az * 0.35)):
                    return True
            return False
        ant_pieces = [p for p in pieces if piece_is_antenna(p)]
        cap_pieces = [p for p in pieces if p not in ant_pieces]
        mat = antenna_material(sc)
        # 先にアンテナをリネーム＋マテリアル付与(Y前→後で番号)
        ant_names = []
        for i, a in enumerate(sorted(ant_pieces, key=lambda o: world_aabb_obj(o)[0].y), 1):
            a.name = "antenna_%d" % i; a.data.name = "antenna_%d" % i
            assign_material(a, mat); ant_names.append(a.name)
        log("本体ピース %d / キャップピース %d" % (len(ant_pieces), len(cap_pieces)))
        # キャップを名前ベースで統合
        join_caps([p.name for p in cap_pieces], ant_names)
        ants = existing_antennas()
        log("分離: %s" % ", ".join(a.name for a in ants))

    if not ants: log("アンテナ無し。終了。"); _dump(); return
    if RESET: reset_prior()
    per = build_rig(ants)
    log("リグ+キーフレーム完了: %s (各%dボーン, 根元固定, frame1→%d)" %
        (ANT_ARM, ANT_BONES, ANIM_END_FRAME))
    ext = ext_config_text(per)
    blend = bpy.data.filepath
    if blend:
        path = os.path.join(os.path.dirname(blend), OUT_TXT)
        try:
            with open(path, "w", encoding="utf-8") as f: f.write(ext + "\n")
            log("ext_configテキスト: %s" % path)
        except Exception as e:
            log("[WARN] 書き出し失敗: %s" % e)
    log("確認: タイムライン再生でアンテナが曲がるか(逆なら ANT_BEND_SIGN=-1, 量=ANT_BEND_SCALE)")
    log("残作業: 原点 Y-up(R+X+90) → FBX → KsEditor(ksSkinnedMesh)")
    _dump()
    print("\n##### ↓ ext_config(任意) ↓ #####"); print(ext); print("##### ここまで #####")

if __name__ == "__main__":
    main()
