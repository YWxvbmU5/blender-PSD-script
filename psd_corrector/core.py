import json
import os
import bpy
import time
import math
from mathutils import Vector, Euler, Quaternion # 导入依赖
from .utils import _safe_name, _get_rest_matrix, _get_pose_matrix, _capture_bone_local_rotation_deg, _capture_bone_local_translation_effective, _capture_bone_local_scale, _is_animation_playing, psd_set_result_datablock_only, psd_set_result_cache_only, psd_get_results_for_arm, psd_set_result_datablock_empty_only, _arm_key_for_obj, psd_flush_mem_to_registered_empty  # 导入捕获函数（如果移动到 utils）
from .utils import PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA, _psd_results_cache
from .json_shape_driver import ShapeDriver
from .json_pose_driver import PoseDriver
from .math_utils import (
    compute_direct_channel_weight, compute_rotation_weight,
    compute_location_weight, compute_scale_weight, compute_triggers
)

# 全局变量
last_compute_time = 0.0
_psd_perf_stats = {}
_psd_bone_state_cache = {}

#计算缓存==================
POST_PROCESS_EXPRESSIONS = {}
POSE_DRIVERS = {}

#============================
# 文件名（可自行修改）
# SHAPE_DRIVERS_FILENAME = "D:\\Desktop\\四元数分解\\shape_drivers.json"
# POSE_DRIVERS_FILENAME = "D:\\Desktop\\四元数分解\\pose_constraints_drivers_export.json"
# # 自动获取当前脚本所在目录（最可靠的方式，插件安装后也能正常工作）
# script_dir = os.path.dirname(os.path.abspath(__file__))
# json_path = os.path.join(script_dir, SHAPE_DRIVERS_FILENAME)
# pose_drivers_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), POSE_DRIVERS_FILENAME)

_shape_expressions_cache = {}   # {arm_key: compiled_dict}
_pose_drivers_cache = {}        # {arm_key: compiled_dict}

# === 修改 load_pose_drivers() 支持新 JSON 结构（多个 property，每个独立 expression）===
def _get_arm_scene(arm_obj):
    """安全获取 arm 所在的 Scene（优先 users_scene）"""
    if arm_obj.users_scene:
        return arm_obj.users_scene[0]
    # fallback（极少情况）
    return bpy.context.scene if bpy.context.scene else list(bpy.data.scenes)[0]

def load_shape_drivers(arm_obj):
    """从 arm_obj 的文件列表加载并编译所有 Shape Driver"""
    if not arm_obj or arm_obj.type != 'ARMATURE':
        return {}
    compiled_data = {}
    loaded_files = 0
    for item in arm_obj.psd_shape_driver_files:
        path = bpy.path.abspath(item.filepath)
        if not os.path.isfile(path):
            print(f"[PSD] Shape Driver 文件不存在: {path}")
            continue
        try:
            with open(path, 'r', encoding='utf-8') as f:
                raw_data = json.load(f)
            # 原编译逻辑（保持不变，仅移到这里）
            for post_key, info in raw_data.items():
                mesh_name = info.get("Mesh_name")
                expr = info.get("expression", "")
                variables = info.get("variables", [])
                if not expr or not variables:
                    continue
                try:
                    compiled = compile(expr, f"<PSD Math: {post_key}>", "eval")
                except Exception as e:
                    print(f"[PSD Math] 表达式编译失败 {post_key}: {e}")
                    continue
                dep_keys = []
                for var in variables:
                    try:
                        var_key = json.loads(var["data_path"])[0]
                        dep_keys.append(var_key)
                    except:
                        pass
                compiled_data[post_key] = {
                    "Mesh_name": mesh_name,
                    "compiled": compiled,
                    "variables": variables,
                    "dep_keys": tuple(sorted(dep_keys))
                }
            loaded_files += 1
        except Exception as e:
            print(f"[PSD] 加载 Shape Driver 失败 {path}: {e}")
    print(f"[PSD Math] 从 {loaded_files} 个文件加载&编译 {len(compiled_data)} 条 Shape Driver")
    return compiled_data

def load_pose_drivers(arm_obj):
    """从 arm_obj 的文件列表加载并编译所有 Pose Driver"""
    if not arm_obj or arm_obj.type != 'ARMATURE':
        return {}
    compiled_data = {}
    loaded_files = 0
    for item in arm_obj.psd_pose_driver_files:
        path = bpy.path.abspath(item.filepath)
        if not os.path.isfile(path):
            print(f"[PSD] Pose Driver 文件不存在: {path}")
            continue
        try:
            with open(path, 'r', encoding='utf-8') as f:
                raw_data = json.load(f)
            # 原编译逻辑（保持不变）
            for bone_name, constraints in raw_data.items():
                bone_dict = compiled_data.setdefault(bone_name, {})
                for constraint_name, prop_dict in constraints.items():
                    constraint_info = bone_dict.setdefault(constraint_name, {})
                    for prop_name, prop_info in prop_dict.items():
                        armature_name = prop_info.get("Armature_name")
                        expr = prop_info.get("expression", "")
                        variables = prop_info.get("variables", [])
                        if not armature_name or not expr or not variables:
                            continue
                        try:
                            compiled = compile(expr, f"<Pose Driver: {bone_name}.{constraint_name}.{prop_name}>", "eval")
                        except Exception as e:
                            print(f"[Pose Driver] 编译失败 {bone_name}.{constraint_name}.{prop_name}: {e}")
                            continue
                        dep_keys = []
                        for var in variables:
                            try:
                                var_key = json.loads(var["data_path"])[0]
                                dep_keys.append(var_key)
                            except:
                                pass
                        constraint_info[prop_name] = {
                            "Armature_name": armature_name,
                            "compiled": compiled,
                            "variables": variables,
                            "dep_keys": tuple(sorted(dep_keys))
                        }
            loaded_files += 1
        except Exception as e:
            print(f"[PSD] 加载 Pose Driver 失败 {path}: {e}")
    total_props = sum(len(c) for b in compiled_data.values() for c in b.values())
    print(f"[Pose Driver] 从 {loaded_files} 个文件加载&编译 {total_props} 条属性驱动")
    return compiled_data

def reload_shape_drivers(arm_obj):
    """强制重新加载并返回条数（供 operator 使用）"""
    arm_key = _arm_key_for_obj(arm_obj)
    new_data = load_shape_drivers(arm_obj)
    _shape_expressions_cache[arm_key] = new_data
    return len(new_data)

def reload_pose_drivers(arm_obj):
    """强制重新加载并返回条数"""
    arm_key = _arm_key_for_obj(arm_obj)
    new_data = load_pose_drivers(arm_obj)
    _pose_drivers_cache[arm_key] = new_data
    total = sum(len(c) for b in new_data.values() for c in b.values())
    return total


def reload_shape_drivers_all():
    total = 0
    for scn in bpy.data.scenes:
        for obj in scn.objects:
            if obj.type == 'ARMATURE':
                arm_key = _arm_key_for_obj(obj)
                new_data = load_shape_drivers(obj)  # 内部已用 _get_arm_scene
                _shape_expressions_cache[arm_key] = new_data
                total += len(new_data)
    return total

def reload_pose_drivers_all():
    total = 0
    for scn in bpy.data.scenes:
        for obj in scn.objects:
            if obj.type == 'ARMATURE':
                arm_key = _arm_key_for_obj(obj)
                new_data = load_pose_drivers(obj)
                _pose_drivers_cache[arm_key] = new_data
                total += len(new_data)
    print(f"[PSD] 全局重新加载 Pose Drivers: {total} 条")
    return total
#============================

# core.py
def psd_get_result_snapshot():
    """返回当前 PSD 计算结果的 dict 快照（线程安全）"""
    return dict(_psd_results_cache)


def init_globals():
    global last_compute_time, _psd_perf_stats, _psd_bone_state_cache
    # global POST_PROCESS_EXPRESSIONS
    # global POSE_DRIVERS

    last_compute_time = 0.0
    _psd_perf_stats = {}
    _psd_bone_state_cache = {}
    # POSE_DRIVERS = load_pose_drivers()

    # # === 新增：插件初始化时只加载一次 JSON（仅此一次）===
    # POST_PROCESS_EXPRESSIONS = load_shape_drivers()
    # if POST_PROCESS_EXPRESSIONS:
    #     print(f"[PSD Math] 插件启动时加载&编译 {len(POST_PROCESS_EXPRESSIONS)} 条表达式 ({json_path})")
    # else:
    #     print("[PSD Math] 插件启动时加载表达式失败或无表达式")
    # # ================================================
    # global shape_driver_instance, pose_driver_instance
    # shape_driver_instance = ShapeDriver(POST_PROCESS_EXPRESSIONS)
    # pose_driver_instance = PoseDriver(POSE_DRIVERS)

def psd_invalidate_bone_cache(arm_name=None, bone_name=None):
    """
    清空缓存：
      - psd_invalidate_bone_cache()             -> 清空全部缓存
      - psd_invalidate_bone_cache(arm_name)     -> 清空某个 arm 的缓存
      - psd_invalidate_bone_cache(arm_name, bn) -> 清空某个 arm 下的某个骨骼缓存
    在添加/删除 saved entry、切换 arm、或手动需要强制刷新时调用。
    """
    global _psd_bone_state_cache
    if arm_name is None:
        _psd_bone_state_cache.clear()
        return
    arm_cache = _psd_bone_state_cache.get(arm_name)
    if not arm_cache:
        return
    if bone_name is None:
        _psd_bone_state_cache.pop(arm_name, None)
    else:
        arm_cache.pop(bone_name, None)

def _psd_make_sample_from_maps(rot_map, loc_map, sca_map, bn, round_ndigits=6):
    """
    从 bone_to_cur_rot / bone_to_cur_loc / bone_to_cur_sca 生成一个可比较的 sample。
    rot_map/loc_map/sca_map 是字典，bn 是骨骼名。
    返回 (loc_tuple, rot_tuple, sca_tuple)，其中每个子项要么是 tuple 要么是 None。
    """
    def _to_tuple(v):
        if v is None:
            return None
        # v 可能是 mathutils.Vector 或 tuple/list
        try:
            return tuple(round(float(x), round_ndigits) for x in v)
        except Exception:
            try:
                return tuple(round(float(x), round_ndigits) for x in tuple(v))
            except Exception:
                return None

    rot_t = _to_tuple(rot_map.get(bn))
    loc_t = _to_tuple(loc_map.get(bn))
    sca_t = _to_tuple(sca_map.get(bn))
    return (loc_t, rot_t, sca_t)


def _psd_samples_equal(s1, s2, eps=1e-5):
    """
    比较两个 sample（三元组）。
    若任一子项为 None 则视为不可比较（返回 False）。
    eps 是每分量允许的最大差值。
    """
    if s1 is None or s2 is None:
        return False
    # s1, s2 都应为 (loc_t, rot_t, sca_t)
    for a, b in zip(s1, s2):
        if a is None or b is None:
            return False
        if len(a) != len(b):
            return False
        for va, vb in zip(a, b):
            if abs(va - vb) > eps:
                return False
    return True

def _psd_compute_all(arm=None, depsgraph=None, scene=None):
    """
    计算所有 armature 的 PSD 结果（修正版，包含稳健的骨骼变换检测缓存）。
    保持权重计算逻辑不变，改变点仅在缓存/跳过未变化骨骼和触发器访问上。
    """
    global last_compute_time, _psd_perf_stats, _psd_bone_state_cache
    current_time = time.time()

    # # 如果未传入 scene，尝试安全获取
    # if scene is None:
    #     if arm and arm.users_scene:
    #         scene = arm.users_scene[0]
    #     elif bpy.context.scene:
    #         scene = bpy.context.scene
    #     else:
    #         scene = list(bpy.data.scenes)[0] if bpy.data.scenes else None
    
    # if not scene or not getattr(scene, 'psd_running', False):
    #     return
    
    # # === 输出模式和 JSON 文件列表现在从 scene 获取（全局）===
    # # mode = getattr(scene, "psd_output_mode", "STORE_TO_EMPTY")
    # mode = getattr(arm, "psd_output_mode", "STORE_TO_EMPTY")
    # # Shape / Pose Driver 加载（从 scene 的全局列表）
    # arm_key = _arm_key_for_obj(arm)
    
    # expressions = _shape_expressions_cache.get(arm_key)
    # if expressions is None:
    #     expressions = load_shape_drivers(arm, scene=scene)  # 修改 load 函数支持 scene
    #     _shape_expressions_cache[arm_key] = expressions
    
    # drivers = _pose_drivers_cache.get(arm_key)
    # if drivers is None:
    #     drivers = load_pose_drivers(arm, scene=scene)
    #     _pose_drivers_cache[arm_key] = drivers


    # 计算最小间隔：播放时不节流 -> min_interval = 0.0；空闲时读取场景属性 psd_idle_hz
    try:
        sc = bpy.context.scene if (bpy.context and getattr(bpy.context, "scene", None)) else None
        if _is_animation_playing():
            min_interval = 0.0
        else:
            hz = int(getattr(sc, "psd_idle_hz", 10) or 10)
            if hz < 1:
                hz = 1
            elif hz > 240:
                hz = 240
            min_interval = 1.0 / float(hz)
    except Exception:
        min_interval = 0.05

    if min_interval > 0.0:
        if current_time - last_compute_time < min_interval:
            return
    last_compute_time = current_time

    # perf flag
    perf_enabled = False
    try:
        sc = bpy.context.scene if bpy.context and bpy.context.scene else None
        perf_enabled = bool(sc and getattr(sc, 'psd_perf_enabled', False))
        history_len = int(getattr(sc, 'psd_perf_history_len', 10) or 10)
    except Exception:
        perf_enabled = False
        history_len = 10

    t_start_total = time.perf_counter() if perf_enabled else None

    # Debug：临时打开以验证缓存行为（不要长期开启）
    DEBUG_CACHE = False

    for arm in [o for o in bpy.data.objects if o.type == 'ARMATURE']:
        # 每个 arm 的处理放在 try/finally 里以保证 arm perf 写回
        try:
            # 可靠的缓存 key（优先 as_pointer()，否则 id(arm)）
            try:
                arm_key = arm.as_pointer()
            except Exception:
                arm_key = id(arm)

            # 如果旧实现/用户之前的缓存是用 arm.name 保存的，迁移它到 arm_key（一次性）
            if arm_key not in _psd_bone_state_cache and getattr(arm, "name", None) in _psd_bone_state_cache:
                try:
                    _psd_bone_state_cache[arm_key] = _psd_bone_state_cache.pop(arm.name)
                except Exception:
                    _psd_bone_state_cache.setdefault(arm_key, {})

            try:
                saved = getattr(arm, 'psd_saved_poses', None)
                if not saved:
                    continue
            except Exception:
                continue

            # bone_filter（和你原来逻辑一致）
            bone_filter = None
            try:
                if getattr(arm, "psd_bone_pairs", None) and len(arm.psd_bone_pairs) > 0:
                    bone_filter = set([p.bone_name for p in arm.psd_bone_pairs if p.bone_name])
            except Exception:
                bone_filter = None
            if bone_filter is None:
                bone_filter = set(e.bone_name for e in saved if e.bone_name)

            # 预采样：当前旋转/位移/缩放（避免重复捕捉）
            bone_to_cur_rot = {}
            bone_to_cur_loc = {}
            bone_to_cur_sca = {}
            source_obj = arm
            try:
                if depsgraph is not None:
                    eval_obj = arm.evaluated_get(depsgraph)
                    if eval_obj:
                        source_obj = eval_obj
            except Exception:
                source_obj = arm

            for bn in bone_filter:
                # 旋转（使用你原有的采样函数）
                try:
                    cur_deg = _capture_bone_local_rotation_deg(arm, bn, depsgraph=depsgraph)
                    if cur_deg is not None:
                        bone_to_cur_rot[bn] = Vector(cur_deg)
                except Exception:
                    pass
                # 位移 / 缩放（从评估对象的 pose.bones 取，或原始 arm 的 pose.bones）
                try:
                    pb = source_obj.pose.bones.get(bn)
                    if pb:
                        # copy() 保证接下来的比较不会被外部修改影响
                        try:
                            cur_loc = _capture_bone_local_translation_effective(arm, bn, depsgraph=depsgraph)
                            bone_to_cur_loc[bn] = cur_loc.copy()
                        except Exception:
                            bone_to_cur_loc[bn] = Vector((0.0, 0.0, 0.0))
                        bone_to_cur_sca[bn] = pb.scale.copy()
                except Exception:
                    pass

            # ----------------- 增加：检测哪些骨骼在本帧没有变化，后面跳过这些骨骼 -----------------
            arm_cache = _psd_bone_state_cache.setdefault(arm_key, {})
            skip_bones = set()
            _PSD_EPS = 1e-5
            for bn_check in bone_filter:
                sample = _psd_make_sample_from_maps(bone_to_cur_rot, bone_to_cur_loc, bone_to_cur_sca, bn_check)
                last_sample = arm_cache.get(bn_check)
                if sample is not None and last_sample is not None and _psd_samples_equal(sample, last_sample, eps=_PSD_EPS):
                    skip_bones.add(bn_check)
                else:
                    # 仅在可采样时写入缓存，避免写入 None
                    if sample is not None:
                        arm_cache[bn_check] = sample

            if DEBUG_CACHE:
                print(f"[PSD CACHE] arm.name={arm.name} arm_key={arm_key} skip {len(skip_bones)} bones; cached={len(arm_cache)}")

            # ------------------------------------------------------------------------------------

            # ----------------- 修复：排除触发器引用的骨骼，不对它们做跳过优化 -----------------
            try:
                trigger_bones = set()
                # 一定要用原始 arm（不要用 eval_obj）去访问自定义 collection
                for trig in getattr(arm, "psd_triggers", ()):
                    # trig.bone_name 是触发器位置来源，trig.target_bone 是被写回结果的目标骨骼
                    if getattr(trig, "bone_name", None):
                        trigger_bones.add(trig.bone_name)
                    if getattr(trig, "target_bone", None):
                        trigger_bones.add(trig.target_bone)
                # 从 skip_bones 中去掉触发器相关骨骼（如果它们存在）
                if trigger_bones:
                    skip_bones.difference_update(trigger_bones)
            except Exception:
                # 忽略任何访问异常，保证稳健性
                pass
            # -----------------------------------------------------------------------------


            # perf container（只有在处理 entries 前才创建）
            arm_stats = None
            if perf_enabled:
                arm_stats = _psd_perf_stats.setdefault(arm.name, {"last_total_ms": 0.0, "last_arm_ms": 0.0, "entries": {}})

            # 记下 arm 层起点（在准备处理 entries 之前）
            t_start_arm = time.perf_counter() if perf_enabled else None

            # ---------- 触发器计算（总是用原始 arm） ----------
            orig_arm = arm
            if getattr(orig_arm, "psd_triggers", None):
                compute_triggers(
                    orig_arm,
                    psd_set_result_cache_only, PREFIX_RESULT_LOC, _safe_name
                )
            #--------------------------------------------------------


            # 遍历 saved entries（按条目计算），但跳过 skip_bones
            for entry in saved:
                bn = entry.bone_name
                en = entry.name
                # 过滤掉没有骨骼名/条目名的条目（这些不是有效样本，不记录 perf）
                if not bn or not en:
                    continue
                if bn not in bone_filter:
                    continue
                # 如果未采样到该骨骼的任何通道则跳过（但仍做 perf 记录）
                if bn not in bone_to_cur_rot and bn not in bone_to_cur_loc:
                    continue

                # 在通过基本有效性检查后开始计时（保证我们不会为无效条目统计）
                t_entry_start = time.perf_counter() if perf_enabled else None

                # 标记是否跳过计算（skip 优化）——但不要直接 continue，使用 flag
                entry_was_skipped = False
                if bn in skip_bones:
                    entry_was_skipped = True

                rep_key = None
                # 用于兜底的 rep_key（即使跳过，也有 key 用于记录）
                try:
                    if getattr(entry, 'has_rot', False):
                        rep_key = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                    elif getattr(entry, 'has_loc', False):
                        rep_key = f"{PREFIX_RESULT_LOC}{_safe_name(bn)}_{_safe_name(en)}"
                    elif getattr(entry, 'has_sca', False):
                        rep_key = f"{PREFIX_RESULT_SCA}{_safe_name(bn)}_{_safe_name(en)}"
                    else:
                        rep_key = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                except Exception:
                    rep_key = f"{PREFIX_RESULT}unknown_{_safe_name(str(en) or 'entry')}"

                try:
                    # 如果跳过，则不做后续重运算；否则按原有逻辑处理 channels
                    if not entry_was_skipped:

                        # ---------------- Direct channel ----------------
                        if getattr(entry, 'is_direct_channel', False):
                            compute_direct_channel_weight(
                                entry, bone_to_cur_rot, arm, bn,
                                psd_set_result_cache_only, PREFIX_RESULT, _safe_name
                            )
                        # ---------------- Rotation channel ----------------
                        if getattr(entry, 'has_rot', False):
                            compute_rotation_weight(
                                entry, bone_to_cur_rot, bn, arm,
                                psd_set_result_cache_only, PREFIX_RESULT, _safe_name
                            )
                        # ---------------- Location channel ----------------
                        if getattr(entry, 'has_loc', False):
                            compute_location_weight(
                                entry, bone_to_cur_loc, bn, arm,
                                psd_set_result_cache_only, PREFIX_RESULT_LOC, _safe_name
                            )
                        # ---------------- Scale channel ----------------
                        if getattr(entry, 'has_sca', False):
                            compute_scale_weight(
                                entry, bone_to_cur_sca, bn, arm,
                                psd_set_result_cache_only, PREFIX_RESULT_SCA, _safe_name
                            )
                        # ---------------- end ----------------
                    else:
                        # 如果 entry_was_skipped：什么也不做（只是跳过 heavy 计算）
                        pass

                except Exception as e_entry:
                    # 记录 entry 层异常（但不阻止 perf 写回）
                    print("PSD entry 计算出错", getattr(entry, "name", "<unknown>"), e_entry)

                finally:
                    # 无论成功/跳过/异常，都尝试记录 entry 时间（如果启用 perf）
                    if perf_enabled and arm_stats is not None:
                        t_entry_end = time.perf_counter()
                        dt_ms = max(0.0, (t_entry_end - t_entry_start) * 1000.0) if t_entry_start else 0.0

                        ent_stats = arm_stats["entries"].setdefault(rep_key, {"hist": [], "last_ms": 0.0, "avg_ms": 0.0})
                        ent_stats["last_ms"] = dt_ms
                        hist = ent_stats["hist"]
                        hist.append(dt_ms)
                        if len(hist) > history_len:
                            hist.pop(0)
                        ent_stats["avg_ms"] = (sum(hist) / len(hist)) if hist else 0.0

            # end for entries

        except Exception as e:
            # 捕获 arm 级别异常（处理过程中可能发生）
            print('PSD计算骨架时出错', getattr(arm, "name", "<unknown>"), e)
        finally:
            # === 批量 flush 内存缓存到持久存储 ===
            arm_key = _arm_key_for_obj(arm)  # utils 中已定义
            mem_cache = _psd_results_cache.get(arm_key, {})

            if mem_cache:
                
                scene = _get_arm_scene(arm)
                mode = getattr(arm, "psd_output_mode", "STORE_TO_EMPTY")

                flushed = False

                if mode == 'STORE_TO_EMPTY':
                    try:
                        # 原有 flush 到 Empty 逻辑（假设 utils 中函数返回 bool 或 None）
                        result = psd_flush_mem_to_registered_empty(arm)
                        flushed = bool(result) if result is not None else True
                        if flushed:
                            # print(f"[PSD] 已将原始结果存储到注册的 Empty（骨架: {arm.name}）")
                            continue
                    except Exception as e:
                        print(f"[PSD] Empty 存储失败: {e}")
                        flushed = False

                    # ================

                elif mode == 'APPLY_DRIVERS':
                    flushed = True
                    # ==================== Shape Driver ====================
                    expressions = _shape_expressions_cache.get(arm_key)
                    if expressions is None:
                        expressions = load_shape_drivers(arm)
                        _shape_expressions_cache[arm_key] = expressions
                    if expressions:
                        ShapeDriver(expressions).process(arm_key, mem_cache)

                    # ==================== Pose Driver ====================
                    drivers = _pose_drivers_cache.get(arm_key)
                    if drivers is None:
                        drivers = load_pose_drivers(arm)
                        _pose_drivers_cache[arm_key] = drivers
                    if drivers:
                        PoseDriver(drivers).process(arm, arm_key, mem_cache)

            # ================================================
                # 如果没有注册 Empty 或 flush 失败，回退批量写到 armature datablock
                #if not flushed:
                #    try:
                #        arm_db = bpy.data.armatures.get(arm.data.name)
                #        if arm_db:
                #            wrote_any = False
                #            for k, v in mem_cache.items():
                #                prev = arm_db.get(k, None)
                #                if prev is None or abs(prev - float(v)) > 1e-6:
                #                    arm_db[k] = float(v)
                #                    wrote_any = True
                #            if wrote_any:
                #                # 可选：只在有变化时 tag（减少 depsgraph 触发）
                #                try:
                #                    arm_db.update_tag()
                #                except Exception:
                #                    pass
                #    except Exception as e:
                #        print("[PSD] fallback datablock 批量写入失败:", e)

                # 可选：计算完后可选择清空该 arm 的内存缓存（节省内存，但下次 minor change 时仍需重写）
                # _psd_results_cache.pop(arm_key, None)
            # arm perf 必须写回（无论 try 是否抛异常）
            if perf_enabled:
                try:
                    t_end_arm = time.perf_counter()
                    arm_ms = max(0.0, (t_end_arm - t_start_arm) * 1000.0) if t_start_arm else 0.0
                    if arm_stats is None:
                        arm_stats = _psd_perf_stats.setdefault(arm.name, {"last_total_ms": 0.0, "last_arm_ms": 0.0, "entries": {}})
                    arm_stats["last_arm_ms"] = arm_ms
                except Exception:
                    # 写 perf 不能让主流程崩溃
                    pass

    # total perf
    if perf_enabled:
        try:
            t_end_total = time.perf_counter()
            total_ms = max(0.0, (t_end_total - t_start_total) * 1000.0) if t_start_total else 0.0
            _psd_perf_stats.setdefault("__global__", {})["last_total_ms"] = total_ms
        except Exception:
            pass

        # === 打印性能统计到控制台（调试用）===
        print("\n=== PSD Performance Stats ===")
        global_stats = _psd_perf_stats.get("__global__", {})
        print(f"Global total: {global_stats.get('last_total_ms', 0.0):.3f} ms")

        for arm_name, stats in _psd_perf_stats.items():
            if arm_name == "__global__":
                continue
            print(f"\nArmature: {arm_name}")
            print(f"  Last arm total: {stats.get('last_arm_ms', 0.0):.3f} ms")
            
            entries = stats.get("entries", {})
            if entries:
                print(f"  Per-entry avg (top 10 slowest):")
                # 按 avg_ms 降序排序，取前10
                sorted_entries = sorted(
                    entries.items(),
                    key=lambda x: x[1].get("avg_ms", 0.0),
                    reverse=True
                )[:10]
                for key, estats in sorted_entries:
                    avg = estats.get("avg_ms", 0.0)
                    last = estats.get("last_ms", 0.0)
                    if avg > 0.01:  # 只显示有意义的
                        print(f"    {key}: avg {avg:.3f} ms | last {last:.3f} ms")
            else:
                print("  No entry stats (all skipped or zero)")
        print("=============================\n")
        # ======================================
