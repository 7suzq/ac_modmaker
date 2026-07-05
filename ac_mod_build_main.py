# -*- coding: utf-8 -*-
"""
ac_mod_build_main.py  ―― 一括実行 ＆ ksanim 自動エクスポート
================================================================================
■ 実行順(step11 DRS は除外):
    ac_mod_build.py (op1-5) → step6_dir → step7_hardpoints → step8_wingflex
    → step9_antenna → step10_sharkfin → step12_uv
    → (最後) ac_mod_tyre_swap.py  ※タイヤ差し替え(外部 2026F1_tyres.blend から)
■ ksanim 自動エクスポート(それぞれ別ファイル, frame1-100):
    fw_armature      → fw_flex.ksanim
    rw_armature      → rw_flex.ksanim
    antenna_armature → antenna.ksanim
    sharkfin_armature→ sharkfin_flex.ksanim
  ※ io_export_ksanim アドオン(bpy.ops.export_scene.ksanim)を使用。有効化必須。

■ -90°X の注意:
  このエクスポーターはボーンを「アーマチュア空間」で書き出す(オブジェクトの
  ワールド回転は無視)。よってオブジェクトを回しても ksanim には反映されない。
  → Armature.transform(Rot(-90°,X)) でレスト骨ごと回転してからエクスポートし、
    直後に逆回転で完全復元する(= 非アーマチュアで exporter が行う [x,z,-y] と一致)。

■ テキスト番号:
  各ステップの [WOBBLY_BIT_n] はローカルに 0 から採番され衝突する。
  → 全 WOBBLY_BIT を通し番号に振り直して _ALL_ext_config.txt に統合。
    WING_/ANIMATION_/DYNAMIC_CONTROLLER_(9,10,11)は一意なので保持し集約。

■ タイヤ差し替え(最後):
  全ステップ完了後に実行。旧 x0_tyre_○○ が WHEEL_○○ の子になっている状態で
  新タイヤがその親を引き継ぐため、ここ(step群の後・エクスポートの前)で行う。
"""
import bpy, os, re
from mathutils import Matrix
from math import radians

# ============================== CONFIG ==============================
# 各ステップ .py が置いてあるフォルダ。空なら __file__ / .blend から自動推定。
SCRIPTS_DIR = "E:/sync/BaiduSyncdisk/ac_modding/convert/blender_script"
# ksanim / 統合iniの出力先。空なら .blend と同じフォルダ。
OUT_DIR = ""

RUN_STEPS      = True     # 各ステップを実行するか
RUN_TYRE_SWAP  = True     # 最後にタイヤ差し替えを実行するか
EXPORT_KSANIM  = True     # ksanim を書き出すか
AGGREGATE_INIS = True     # WOBBLY_BIT 等を通し番号で統合するか

EXPORT_ROT_X_DEG = -90.0  # エクスポート時のレスト骨回転(X)。このファイルの座標系では -90。

STEP_FILES = [
    "ac_mod_build.py",
    "ac_mod_step6_dir.py",
    "ac_mod_step7_hardpoints.py",
    "ac_mod_step8_wingflex.py",
    "ac_mod_step9_antenna.py",
    "ac_mod_step10_sharkfin.py",
    "ac_mod_step12_uv.py",
]
# タイヤ差し替えスクリプト(最後に単独実行)。2026F1_tyres.blend も同じ SCRIPTS_DIR に置く。
TYRE_SWAP_FILE = "ac_mod_tyre_swap.py"

# (アーマチュア名, ksanimファイル名, frame_start, frame_end)
KSANIM_EXPORTS = [
    ("fw_armature",       "fw_flex.ksanim",       1, 100),
    ("rw_armature",       "rw_flex.ksanim",       1, 100),
    ("antenna_armature",  "antenna.ksanim",       1, 100),
    ("sharkfin_armature", "sharkfin_flex.ksanim", 1, 100),
]

# 統合対象の各ステップ出力テキスト(.blend フォルダに生成される)
INI_SOURCES = ["_wingflex_inis.txt", "_antenna_ext_config.txt", "_sharkfin_inis.txt"]
# ===================================================================

REPORT = []
def log(s): REPORT.append(str(s)); print(s)

def blend_dir():
    return os.path.dirname(bpy.data.filepath) if bpy.data.filepath else ""

def resolve_scripts_dir():
    if SCRIPTS_DIR: return SCRIPTS_DIR
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except NameError:
        return blend_dir()

def resolve_out_dir():
    return OUT_DIR if OUT_DIR else blend_dir()

def to_object_mode():
    obj = bpy.context.view_layer.objects.active
    if obj and obj.mode != 'OBJECT':
        try: bpy.ops.object.mode_set(mode='OBJECT')
        except Exception: pass

# ---------------------------------------------------------------- exec 共通
def exec_file(fn):
    """SCRIPTS_DIR 内の .py を __name__='__main__' で実行(= main() を走らせる)。"""
    sdir = resolve_scripts_dir()
    path = os.path.join(sdir, fn)
    if not os.path.isfile(path):
        log("[SKIP] 見つからない: %s" % path); return
    log("\n===== 実行: %s =====" % fn)
    to_object_mode()
    try: bpy.ops.object.select_all(action='DESELECT')
    except Exception: pass
    try: bpy.context.scene.frame_set(1)
    except Exception: pass
    try:
        with open(path, "r", encoding="utf-8") as f:
            src = f.read()
        g = {"__name__": "__main__", "__file__": path}
        exec(compile(src, path, "exec"), g)
        sub = g.get("REPORT")
        if isinstance(sub, list) and sub:
            REPORT.append("  ┄┄ %s 詳細ログ ┄┄" % fn)
            for line in sub:
                REPORT.append("    " + str(line))   # 既にstepがprint済み→ここは再printせず統合のみ
        log("----- %s 完了 -----" % fn)
    except Exception as e:
        import traceback
        log("[ERR] %s: %s" % (fn, e))
        traceback.print_exc()

# ---------------------------------------------------------------- steps
def run_steps():
    log("スクリプトフォルダ: %s" % (resolve_scripts_dir() or "(不明)"))
    for fn in STEP_FILES:
        exec_file(fn)
    to_object_mode()
    try: bpy.context.scene.frame_set(1)
    except Exception: pass

def run_tyre_swap():
    log("\n======== タイヤ差し替え(最後) ========")
    exec_file(TYRE_SWAP_FILE)
    to_object_mode()

def check_outputs():
    """期待される生成物(アーマチュア等)が出来ているか確認し、未生成を明示。"""
    log("\n======== 生成物チェック ========")
    expect = [
        ("fw_armature",       "フロントウイングflex", "ac_mod_step8_wingflex.py"),
        ("rw_armature",       "リアウイングflex",     "ac_mod_step8_wingflex.py"),
        ("antenna_armature",  "アンテナ",             "ac_mod_step9_antenna.py"),
        ("sharkfin_armature", "シャークフィン",       "ac_mod_step10_sharkfin.py"),
    ]
    any_miss = False
    for name, label, step in expect:
        o = bpy.data.objects.get(name)
        if o and o.type == 'ARMATURE':
            log("[OK] %s : %s" % (name, label))
        else:
            any_miss = True
            log("[未生成] %s : %s → 上の『%s 詳細ログ』の[診断]行で原因を確認してください。"
                % (name, label, step))
    if not any_miss:
        log("すべて生成済み。")

# ---------------------------------------------------------------- ksanim
def ksanim_available():
    try:
        return hasattr(bpy.ops.export_scene, "ksanim")
    except Exception:
        return False

def export_all_ksanim():
    if not ksanim_available():
        log("[ksanim] アドオン(io_export_ksanim)が無効です。File>Export に "
            "'Assetto Corsa animation (.ksanim)' が出るよう有効化してください。")
        return
    out = resolve_out_dir()
    if not out:
        log("[ksanim] .blend 未保存 → 出力先不明。保存後に再実行してください。"); return
    scene = bpy.context.scene
    f0, f1 = scene.frame_start, scene.frame_end
    Rx = Matrix.Rotation(radians(EXPORT_ROT_X_DEG), 4, 'X')
    Rxi = Rx.inverted()
    for arm_name, fname, fs, fe in KSANIM_EXPORTS:
        obj = bpy.data.objects.get(arm_name)
        if not obj or obj.type != 'ARMATURE':
            log("[ksanim] %s 無し → スキップ" % arm_name); continue
        to_object_mode()
        try: obj.hide_set(False)
        except Exception: pass
        obj.hide_viewport = False; obj.hide_select = False
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        scene.frame_start = fs; scene.frame_end = fe
        path = os.path.join(out, fname)
        # レスト骨ごと -90°X 回転 → エクスポート → 逆回転で復元
        obj.data.transform(Rx)
        bpy.context.view_layer.update()
        ok = False
        try:
            res = bpy.ops.export_scene.ksanim(
                filepath=path, selection_type='use_selection',
                reverse_animation=False, add_colons=False, export_base_pos=False)
            ok = ('FINISHED' in res)
        except Exception as e:
            log("[ksanim] %s 失敗: %s" % (fname, e))
        finally:
            obj.data.transform(Rxi)
            bpy.context.view_layer.update()
        if ok:
            log("[ksanim] %s → %s (frame %d-%d)" % (arm_name, fname, fs, fe))
    scene.frame_start = f0; scene.frame_end = f1
    scene.frame_set(1)

# ---------------------------------------------------------------- ini 統合
def parse_sections(text):
    """[SECTION] 単位に分割。戻り: [(header or None, [body_lines]), ...]"""
    out = []; head = None; body = []
    for ln in text.splitlines():
        if re.match(r'^\s*\[[^\]]+\]\s*$', ln):
            out.append((head, body)); head = ln.strip(); body = []
        else:
            body.append(ln)
    out.append((head, body))
    return out

def aggregate_inis():
    bd = blend_dir()
    if not bd:
        log("[ini] .blend 未保存 → 統合スキップ"); return
    wob, aero, anim = [], [], []
    wi = 0
    for src in INI_SOURCES:
        p = os.path.join(bd, src)
        if not os.path.isfile(p): continue
        with open(p, "r", encoding="utf-8") as f:
            secs = parse_sections(f.read())
        for head, body in secs:
            if head is None: continue
            name = head.strip("[]")
            block_body = "\n".join(body).rstrip()
            if name.upper().startswith("WOBBLY_BIT"):
                wob.append("[WOBBLY_BIT_%d]\n%s" % (wi, block_body)); wi += 1
            elif name.upper().startswith("WING") or name.upper().startswith("DYNAMIC_CONTROLLER"):
                aero.append("%s\n%s" % (head, block_body))
            elif name.upper().startswith("ANIMATION"):
                anim.append("%s\n%s" % (head, block_body))
    out = resolve_out_dir() or bd
    def dump(fn, header, blocks):
        if not blocks: return
        path = os.path.join(out, fn)
        with open(path, "w", encoding="utf-8") as f:
            f.write(header + "\n\n" + "\n\n".join(blocks) + "\n")
        log("[ini] %s (%d ブロック)" % (fn, len(blocks)))
    dump("_ALL_ext_config.txt",
         "; ===== ext_config.ini に追記(WOBBLY_BITは通し番号に振り直し済) =====", wob)
    dump("_ALL_aero.txt",
         "; ===== aero.ini に追記(WING_/DYNAMIC_CONTROLLER_ ; 番号は各ステップ準拠) =====", aero)
    dump("_ALL_wing_animation.txt",
         "; ===== wing_animation.ini に追記(ANIMATION_) =====", anim)
    if wob:
        log("[ini] WOBBLY_BIT 合計 %d 個を 0..%d に再採番" % (wi, wi - 1))

# ---------------------------------------------------------------- main
def main():
    log("################ ac_mod_build_main ################")
    if RUN_STEPS:      run_steps()
    if RUN_TYRE_SWAP:  run_tyre_swap()          # ← step群の後、エクスポートの前(最後のシーン編集)
    if RUN_STEPS:      check_outputs()          # 期待生成物(アーマチュア)の有無を明示
    if EXPORT_KSANIM:  log("\n======== ksanim エクスポート ========"); export_all_ksanim()
    if AGGREGATE_INIS: log("\n======== ini テキスト統合 ========"); aggregate_inis()
    log("\n残作業(手動): 車体ジオメトリの FBX 出力 → KsEditor で ksSkinnedMesh 割当 → "
        "各 *_anim.lut / Neutral.lut を car の data フォルダへ配置。")
    log("###################################################")
    print("\n##### COPY (マスターレポート) #####")
    print("\n".join(REPORT))

if __name__ == "__main__":
    main()
