import bpy
import time
import math
from mathutils import Vector, Euler, Quaternion # 导入依赖
from .math_utils import _euler_deg_to_quat, _swing_twist_decompose, _signed_angle_from_quat, _triangular_ratio,  _euler_deg_to_dir # 导入数学工具
from .utils import _safe_name, _get_rest_matrix, _get_pose_matrix, _capture_bone_local_rotation_deg, _capture_bone_local_translation, _capture_bone_local_scale, _is_animation_playing, psd_set_result_datablock_only  # 导入捕获函数（如果移动到 utils）
from .utils import PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA

# 全局变量
last_compute_time = 0.0
_psd_perf_stats = {}
_psd_bone_state_cache = {}

def init_globals():
    global last_compute_time, _psd_perf_stats, _psd_bone_state_cache
    last_compute_time = 0.0
    _psd_perf_stats = {}
    _psd_bone_state_cache = {}

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

def _psd_compute_all(depsgraph=None):
    """
    计算所有 armature 的 PSD 结果（修正版，包含稳健的骨骼变换检测缓存）。
    保持权重计算逻辑不变，改变点仅在缓存/跳过未变化骨骼和触发器访问上。
    """
    global last_compute_time, _psd_perf_stats, _psd_bone_state_cache
    current_time = time.time()

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
                        bone_to_cur_loc[bn] = pb.location.copy()
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


            # perf container
            arm_stats = None
            if perf_enabled:
                arm_stats = _psd_perf_stats.setdefault(arm.name, {"last_total_ms": 0.0, "last_arm_ms": 0.0, "entries": {}})

            t_start_arm = time.perf_counter() if perf_enabled else None

            # 遍历 saved entries（按条目计算），但跳过 skip_bones
            for entry in saved:
                bn = entry.bone_name
                en = entry.name
                if not bn or not en:
                    continue
                if bn not in bone_filter:
                    continue
                # 如果未采样到该骨骼的任何通道则跳过
                if bn not in bone_to_cur_rot and bn not in bone_to_cur_loc:
                    continue
                # 跳过本帧被判定为未变化的骨骼
                if bn in skip_bones:
                    continue

                t_entry_start = time.perf_counter() if perf_enabled else None

                # ---------------- Direct channel ----------------
                if getattr(entry, 'is_direct_channel', False):
                    try:
                        key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                        cur_rot = bone_to_cur_rot.get(bn)
                        # 如果没有采样到旋转角度，则设置为零。
                        if cur_rot is None:
                            psd_set_result_datablock_only(arm, key_rot, 0.0, verbose=False)
                        else:
                            mode = getattr(entry, 'record_rot_channel_mode', 'NONE')
                            ch_axis = getattr(entry, 'channel_axis', 'X')
                            axis_idx_map = {'X': 0, 'Y': 1, 'Z': 2}
                            axis_idx = axis_idx_map.get(ch_axis, 0)
                            cur_rot_value = float(cur_rot[axis_idx])
                            q_cur = _euler_deg_to_quat(tuple(cur_rot))

                            # 默认：使用原始欧拉角值（以度为单位）
                            w_deg = cur_rot_value

                        #========================================================
                        #=   我建议使用(ZXY)这个旋转次序。因为我喜欢Y是向上的，哥们。
                        #========================================================

                            # 处理 X/Y/Z 扭转轴的摆动-扭转直接记录
                            if mode in ('record_rot_SWING_X_TWIST', 'record_rot_SWING_Y_TWIST', 'record_rot_SWING_Z_TWIST'):
                                # 从模式名称中选择扭转轴
                                if mode == 'record_rot_SWING_X_TWIST':
                                    twist_axis = Vector((1.0, 0.0, 0.0))
                                elif mode == 'record_rot_SWING_Y_TWIST':
                                    twist_axis = Vector((0.0, 1.0, 0.0))
                                else:
                                    twist_axis = Vector((0.0, 0.0, 1.0))

                                # 分解为摆动和扭转（math_utils 库会在内部处理归一化）。
                                swing_q, twist_q = _swing_twist_decompose(q_cur, twist_axis)

                                # 安全夹具辅助工具
                                def _clamp1(v):
                                    if v is None:
                                        return 0.0
                                    if v > 1.0:
                                        return 1.0
                                    if v < -1.0:
                                        return -1.0
                                    return float(v)

                                # 提取通道的度数：
                                if ch_axis == 'X':
                                    # swing.x -> angle = 2 * asin(swing.x)
                                    sx = _clamp1(getattr(swing_q, 'x', 0.0))
                                    w_deg = math.degrees(2.0 * math.asin(sx))
                                elif ch_axis == 'Y':
                                    # 绕旋转轴扭转：以度为单位的有符号角度
                                    w_deg = _signed_angle_from_quat(twist_q, twist_axis)
                                elif ch_axis == 'Z':
                                    sz = _clamp1(getattr(swing_q, 'z', 0.0))
                                    w_deg = math.degrees(2.0 * math.asin(sz))
                                else:
                                    w_deg = cur_rot_value

                            # 转换为弧度进行存储（以保持与现有存储格式的兼容性）
                            w = math.radians(w_deg)
                            if math.isnan(w):
                                w = 0.0
                            psd_set_result_datablock_only(arm, key_rot, w, verbose=False)
                    except Exception as e:
                        try:
                            key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                            psd_set_result_datablock_only(arm, key_rot, 0.0, verbose=False)
                        except Exception:
                            pass

                # ---------------- Rotation channel ----------------
                if getattr(entry, 'has_rot', False):
                    try:
                        cur_rot = bone_to_cur_rot.get(bn)
                        if cur_rot is not None:
                            if getattr(entry, 'cone_enabled', False):
                                dir_center = _euler_deg_to_dir(entry.pose_rot, entry.cone_axis)
                                dir_cur = _euler_deg_to_dir(tuple(cur_rot), entry.cone_axis)
                                dot = max(-1.0, min(1.0, dir_center.dot(dir_cur)))
                                angle_rad = math.acos(dot)
                                angle_deg = math.degrees(angle_rad)
                                if angle_deg <= entry.cone_angle:
                                    w = 1.0 - (angle_deg / entry.cone_angle) if entry.cone_angle > 0.0 else 1.0
                                else:
                                    w = 0.0
                            elif getattr(entry, 'rot_channel_mode', 'NONE') and entry.rot_channel_mode != 'NONE':
                                try:
                                    # 保持原算法：用轴分量差做三角包络
                                    axis_idx_map = {'SWING_X_TWIST': 0, 'SWING_Y_TWIST': 1, 'SWING_Z_TWIST': 2}
                                    axis_idx = axis_idx_map.get(entry.rot_channel_mode, 2)
                                    target_twist_angle = float(entry.pose_rot[axis_idx] - entry.rest_rot[axis_idx])
                                    cur_twist_angle = float(cur_rot[axis_idx] - entry.rest_rot[axis_idx])
                                    w_twist = _triangular_ratio(cur_twist_angle, target_twist_angle)
                                    w = float(max(0.0, min(1.0, w_twist)))
                                except Exception:
                                    pose_dir = Vector(entry.pose_rot) - Vector(entry.rest_rot)
                                    cur_rel = Vector(cur_rot) - Vector(entry.rest_rot)
                                    denom = pose_dir.dot(pose_dir)
                                    if denom < 1e-6:
                                        w = 1.0 if cur_rel.length < 1e-3 else 0.0
                                    else:
                                        w = cur_rel.dot(pose_dir) / denom
                                        if w < 0.0:
                                            w = 0.0
                                        elif w > 2.0:
                                            w = 0.0
                                        elif w > 1.0:
                                            w = 2.0 - w
                            else:
                                pose_dir = Vector(entry.pose_rot) - Vector(entry.rest_rot)
                                cur_rel = cur_rot - Vector(entry.rest_rot)
                                denom = pose_dir.dot(pose_dir)
                                if denom < 1e-6:
                                    w = 1.0 if cur_rel.length < 1e-3 else 0.0
                                else:
                                    w = cur_rel.dot(pose_dir) / denom
                                    if w < 0.0:
                                        w = 0.0
                                    elif w > 2.0:
                                        w = 0.0
                                    elif w > 1.0:
                                        w = 2.0 - w
                            if math.isnan(w):
                                w = 0.0
                            w = max(0.0, min(1.0, w))
                        else:
                            w = 0.0
                    except Exception:
                        w = 0.0

                    key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                    try:
                        psd_set_result_datablock_only(arm, key_rot, w, verbose=False)
                    except Exception:
                        pass

                # ---------------- Location channel ----------------
                if getattr(entry, 'has_loc', False):
                    try:
                        cur_loc = bone_to_cur_loc.get(bn)
                        if cur_loc is not None:
                            center = Vector(entry.pose_loc)
                            radius = float(getattr(entry, 'loc_radius', 0.0) or 0.0)
                            if getattr(entry, 'loc_enabled', False):
                                if radius <= 0.0:
                                    w_loc = 1.0 if (cur_loc - center).length < 1e-6 else 0.0
                                else:
                                    d = cur_loc - center
                                    wx = max(0.0, 1.0 - abs(d.x) / radius)
                                    wy = max(0.0, 1.0 - abs(d.y) / radius)
                                    wz = max(0.0, 1.0 - abs(d.z) / radius)
                                    w_loc = wx * wy * wz
                            else:
                                w_loc = 1.0
                                num_nonzero = 0
                                rest_vec = Vector(getattr(entry, 'rest_loc', (0.0, 0.0, 0.0)))
                                pose_vec = Vector(getattr(entry, 'pose_loc', (0.0, 0.0, 0.0)))
                                direction = pose_vec - rest_vec
                                len2 = direction.length_squared
                                if len2 < 1e-12:
                                    w_loc = 1.0 if (cur_loc - rest_vec).length < 1e-6 else 0.0
                                else:
                                    t = (cur_loc - rest_vec).dot(direction) / len2
                                    if math.isnan(t):
                                        t = 0.0
                                    if t < 0.0 or t > 2.0:
                                        w_loc = 0.0
                                    elif t > 1.0:
                                        w_loc = 2.0 - t
                                    else:
                                        w_loc = t
                                    w_loc = max(0.0, min(1.0, w_loc))
                                    num_nonzero += 1
                                if num_nonzero == 0:
                                    w_loc = 1.0 if (cur_loc - rest_vec).length < 1e-6 else 0.0
                            if math.isnan(w_loc):
                                w_loc = 0.0
                            w_loc = max(0.0, min(1.0, w_loc))
                        else:
                            w_loc = 0.0
                    except Exception:
                        w_loc = 0.0

                    key_loc = f"{PREFIX_RESULT_LOC}{_safe_name(bn)}_{_safe_name(en)}"
                    try:
                        psd_set_result_datablock_only(arm, key_loc, w_loc, verbose=False)
                    except Exception:
                        pass

                # ---------------- Scale channel ----------------
                if getattr(entry, 'has_sca', False):
                    try:
                        cur_sca = bone_to_cur_sca.get(bn)
                        if cur_sca is not None:
                            rest_vec = Vector(getattr(entry, 'rest_sca', (1.0, 1.0, 1.0)))
                            pose_vec = Vector(getattr(entry, 'pose_sca', (1.0, 1.0, 1.0)))
                            direction = pose_vec - rest_vec
                            len2 = direction.length_squared
                            if len2 < 1e-12:
                                w_sca = 1.0 if (cur_sca - rest_vec).length < 1e-6 else 0.0
                            else:
                                t = (cur_sca - rest_vec).dot(direction) / len2
                                if math.isnan(t):
                                    t = 0.0
                                if t < 0.0 or t > 2.0:
                                    w_sca = 0.0
                                elif t > 1.0:
                                    w_sca = 2.0 - t
                                else:
                                    w_sca = t
                                w_sca = max(0.0, min(1.0, w_sca))
                            if math.isnan(w_sca):
                                w_sca = 0.0
                            w_sca = max(0.0, min(1.0, w_sca))
                        else:
                            w_sca = 0.0
                    except Exception:
                        w_sca = 0.0

                    key_sca = f"{PREFIX_RESULT_SCA}{_safe_name(bn)}_{_safe_name(en)}"
                    try:
                        psd_set_result_datablock_only(arm, key_sca, w_sca, verbose=False)
                    except Exception:
                        pass

                # ---------- 触发器计算（总是用原始 arm） ----------
                orig_arm = arm
                if getattr(orig_arm, "psd_triggers", None):
                    for trig in orig_arm.psd_triggers:
                        trig.last_weight = 0.0
                        if not trig.enabled:
                            continue
                        pb_trigger = orig_arm.pose.bones.get(trig.bone_name)
                        pb_target = orig_arm.pose.bones.get(trig.target_bone)
                        if not pb_trigger or not pb_target:
                            continue
                        try:
                            head_trigger_world = orig_arm.matrix_world @ pb_trigger.head
                            head_target_world = orig_arm.matrix_world @ pb_target.head
                        except Exception:
                            head_trigger_world = orig_arm.matrix_world @ pb_trigger.bone.head_local
                            head_target_world = orig_arm.matrix_world @ pb_target.bone.head_local

                        d = (head_target_world - head_trigger_world).length
                        r = max(1e-6, float(trig.radius))
                        if trig.falloff == 'SMOOTH':
                            t = min(max(d / r, 0.0), 1.0)
                            w = 1.0 - (3.0 * t * t - 2.0 * t * t * t)
                        else:
                            w = max(0.0, min(1.0, 1.0 - d / r))
                        trig.last_weight = w
                        key_base = f"{PREFIX_RESULT_LOC}{_safe_name(trig.target_bone)}_{_safe_name(trig.name)}"
                        try:
                            psd_set_result_datablock_only(orig_arm, f"{key_base}_w", float(w), verbose=False)
                        except Exception:
                            pass

                # 性能记录
                if perf_enabled:
                    t_entry_end = time.perf_counter()
                    dt_ms = max(0.0, (t_entry_end - t_entry_start) * 1000.0) if t_entry_start else 0.0
                    rep_key = None
                    if getattr(entry, 'has_rot', False):
                        rep_key = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                    elif getattr(entry, 'has_loc', False):
                        rep_key = f"{PREFIX_RESULT_LOC}{_safe_name(bn)}_{_safe_name(en)}"
                    elif getattr(entry, 'has_sca', False):
                        rep_key = f"{PREFIX_RESULT_SCA}{_safe_name(bn)}_{_safe_name(en)}"
                    else:
                        rep_key = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                    ent_stats = arm_stats["entries"].setdefault(rep_key, {"hist": [], "last_ms": 0.0, "avg_ms": 0.0})
                    ent_stats["last_ms"] = dt_ms
                    hist = ent_stats["hist"]
                    hist.append(dt_ms)
                    if len(hist) > history_len:
                        hist.pop(0)
                    ent_stats["avg_ms"] = (sum(hist) / len(hist)) if hist else 0.0

            # arm perf
            if perf_enabled:
                t_end_arm = time.perf_counter()
                arm_ms = max(0.0, (t_end_arm - t_start_arm) * 1000.0) if t_start_arm else 0.0
                arm_stats["last_arm_ms"] = arm_ms

        except Exception as e:
            print('PSD计算骨架时出错', getattr(arm, "name", "<unknown>"), e)

    # total perf
    if perf_enabled:
        t_end_total = time.perf_counter()
        total_ms = max(0.0, (t_end_total - t_start_total) * 1000.0) if t_start_total else 0.0
        _psd_perf_stats.setdefault("__global__", {})["last_total_ms"] = total_ms


