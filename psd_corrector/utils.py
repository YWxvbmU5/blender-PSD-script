import bpy
import re
import math
from mathutils import Vector, Euler, Quaternion  # 如果需要

# 注册属性名（保存到 Armature datablock）
_PSD_CACHE_OBJ_PROP = "_psd_cache_obj"

PREFIX_RESULT = "psd_rot_"  
PREFIX_RESULT_LOC = "psd_loc_"  
PREFIX_RESULT_SCA = "psd_sca_"  

# 内存缓存：{ arm_key -> { key_str -> float_value, ... }, ... }
_psd_results_cache = {}

#======================================================================

def psd_register_cache_empty(obj_arm: bpy.types.Object, empty_obj: bpy.types.Object, verbose=False) -> bool:
    """
    将 empty_obj 注册为 obj_arm 的 PSD 缓存存储对象（持久化到 armature datablock）。
    必须保证 obj_arm.type == 'ARMATURE' 且 empty_obj.type == 'EMPTY'。
    """
    if not obj_arm or obj_arm.type != 'ARMATURE':
        if verbose: print("[PSD] psd_register_cache_empty: obj_arm 不是骨骼对象")
        return False
    if not empty_obj or empty_obj.type != 'EMPTY':
        if verbose: print("[PSD] psd_register_cache_empty: empty_obj 不是 Empty")
        return False

    arm_db = bpy.data.armatures.get(obj_arm.data.name)
    if not arm_db:
        if verbose: print("[PSD] psd_register_cache_empty: 找不到 armature datablock")
        return False

    try:
        arm_db[_PSD_CACHE_OBJ_PROP] = empty_obj.name
        if verbose:
            print(f"[PSD] 注册缓存 Empty '{empty_obj.name}' 到骨架 datablock '{arm_db.name}'（属性: {_PSD_CACHE_OBJ_PROP}）")
        return True
    except Exception as e:
        if verbose:
            print("[PSD] psd_register_cache_empty 错误:", e)
        return False

def psd_unregister_cache_empty(obj_arm: bpy.types.Object, verbose=False) -> bool:
    """移除注册的缓存 Empty（如果存在）。"""
    if not obj_arm or obj_arm.type != 'ARMATURE':
        if verbose: print("[PSD] psd_unregister_cache_empty: obj_arm 不是骨骼对象")
        return False
    arm_db = bpy.data.armatures.get(obj_arm.data.name)
    if not arm_db:
        if verbose: print("[PSD] psd_unregister_cache_empty: 找不到 armature datablock")
        return False
    if _PSD_CACHE_OBJ_PROP in arm_db:
        try:
            del arm_db[_PSD_CACHE_OBJ_PROP]
            if verbose:
                print(f"[PSD] 已移除骨架 datablock '{arm_db.name}' 的 {_PSD_CACHE_OBJ_PROP}")
            return True
        except Exception as e:
            if verbose: print("[PSD] psd_unregister_cache_empty 错误:", e)
            return False
    if verbose: print("[PSD] psd_unregister_cache_empty: 未设置缓存 Empty")
    return False

def psd_get_registered_empty(obj_arm: bpy.types.Object):
    """
    返回注册的 Empty 对象（或 None）。非常快速（只读 datablock 的单个字符串并做一次 bpy.data.objects.get）。
    """
    if not obj_arm or obj_arm.type != 'ARMATURE':
        return None
    arm_db = bpy.data.armatures.get(obj_arm.data.name)
    if not arm_db:
        return None
    name = arm_db.get(_PSD_CACHE_OBJ_PROP, None)
    if not name:
        return None
    return bpy.data.objects.get(name)

def psd_set_result_to_registered_empty(obj_arm: bpy.types.Object, key: str, value, verbose=False) -> bool:
    """
    将 value 写入已注册到 obj_arm 的 Empty 的自定义属性中（高频写入用）。
    - 不会创建 Empty，也不会调用 update_tag()
    - 如果未注册或 Empty 已被删除，返回 False
    - 仅在值发生“实质变化”时写入（避免多余写入）
    """
    try:
        fw = float(value)
    except Exception:
        return False

    cache_obj = psd_get_registered_empty(obj_arm)
    if cache_obj is None:
        if verbose:
            print("[PSD] 未注册缓存 Empty，或已被删除（_psd_cache_obj 未设置或无效）")
        return False

    prev = cache_obj.get(key, None)
    if prev is None or abs(prev - fw) > 1e-6:
        cache_obj[key] = fw
        if verbose:
            print(f"[PSD] 已写入缓存 Empty '{cache_obj.name}': {key} = {fw}")
        return True
    return False


def psd_flush_mem_to_registered_empty(obj_arm: bpy.types.Object, verbose=False) -> bool:
    """
    把当前内存缓存（_psd_results_cache）批量写回到已注册 Empty（一次性批量写入）。
    返回 True 如果有写入发生。
    """
    try:
        arm_key = _arm_key_for_obj(obj_arm)
        mem = _psd_results_cache.get(arm_key)
        if not mem:
            return False

        cache_obj = psd_get_registered_empty(obj_arm)
        if cache_obj is None:
            if verbose:
                print("[PSD] flush failed: 未注册缓存 Empty")
            return False

        # 收集实际变化的属性（优化：只写变化的）
        changes = {}
        for k, v in mem.items():
            fv = float(v)
            prev = cache_obj.get(k, None)
            if prev is None or abs(prev - fv) > 0.001:
                changes[k] = fv

        if not changes:
            return False  # 无变化，直接返回

        # 批量赋值（仍是循环，但这是最快方式）
        for k, v in changes.items():
            cache_obj[k] = v

        # 只在有变化时 tag（关键！减少 depsgraph 触发）
        try:
            cache_obj.update_tag()
        except Exception as e:
            if verbose:
                print("[PSD] cache_obj.update_tag() 失败:", e)

        if verbose:
            print(f"[PSD] 已把内存缓存写回 Empty '{cache_obj.name}'（实际写入 {len(changes)} 条）")

        return True
    except Exception as e:
        if verbose:
            print("[PSD] psd_flush_mem_to_registered_empty 错误:", e)
        return False


def psd_set_result_datablock_empty_only(obj_arm, key, value, verbose=False):
    """
    优先写入用户注册的 Empty（如果注册了），否则回退到写入 Armature datablock（保留旧行为）。
    尽量避免调用 update_tag() 以减少评估开销。
    """
    try:
        fw = float(value)
    except Exception:
        return False

    # 优先写到注册的 empty（如果有）
    try:
        cache_obj = psd_get_registered_empty(obj_arm)
        if cache_obj:
            prev = cache_obj.get(key, None)
            if prev is None or abs(prev - fw) > 1e-6:
                cache_obj[key] = fw
                if verbose:
                    print(f"[PSD] 已写入注册 Empty '{cache_obj.name}': {key} = {fw}")
                return True
            #try:
            #    cache_obj.update_tag()
            #except Exception:
            #    pass
            return False
    except Exception:
        # 忽略并回退到 datablock 写入
        pass

    # 如果没有注册 Empty，则回退到写入 Armature datablock（原行为）
    wrote = False
    try:
        arm_db = bpy.data.armatures.get(obj_arm.data.name)
        if arm_db is not None:
            prev = arm_db.get(key, None)
            if prev is None or abs(prev - fw) > 1e-6:
                arm_db[key] = fw
                wrote = True
                if verbose:
                    print(f"[PSD] 已写入骨架数据块 '{arm_db.name}': {key} = {fw}")
            #try:
            #    arm_db.update_tag()
            #except Exception:
            #    pass
            # 注意：不再显式调用 arm_db.update_tag()（避免强制 depsgraph 更新）
    except Exception as e:
        if verbose:
            print("psd_set_result_datablock_only: 写入失败:", e)
        return False
    return wrote


#============================================================================

def _arm_key_for_obj(obj_arm):
    try:
        return obj_arm.as_pointer()
    except Exception:
        return id(obj_arm)

def psd_set_result_cache_only(obj_arm, key, value, verbose=False):
    """
    把计算结果只写入内存缓存（不改 datablock）。
    返回 True 如果实际写入（新值或改变）。
    """
    try:
        fw = float(value)
    except Exception:
        return False

    try:
        arm_key = _arm_key_for_obj(obj_arm)
        arm_cache = _psd_results_cache.setdefault(arm_key, {})
        prev = arm_cache.get(key, None)
        if prev is None or abs(prev - fw) > 0.001:
            arm_cache[key] = fw
            if verbose:
                print(f"[PSD CACHE] 写入缓存 {obj_arm.name} : {key} = {fw}")

            # ===== 新增：把更新后的 snapshot 推给 concurrent_mapping =====
            #try:
            #    # 局部导入，避免循环依赖
            #    from .concurrent_mapping import schedule_parallel_eval

                # ⚠️ 关键点：传的是「纯 dict 快照」
            #    snapshot = dict(arm_cache)
            #    schedule_parallel_eval(snapshot)

            #except Exception as e:
            #    if verbose:
            #        print("[PSD CACHE] push to concurrent_mapping failed:", e)
            # ======================================================
            return True
    except Exception as e:
        if verbose:
            print("psd_set_result_cache_only error:", e)
        return False
    return False

def psd_get_results_for_arm(obj_arm):
    """返回该 arm 当前缓存的所有计算结果的浅拷贝字典（key->float）。"""
    arm_key = _arm_key_for_obj(obj_arm)
    return dict(_psd_results_cache.get(arm_key, {}))

def psd_get_result(obj_arm, key, default=None):
    """按 key 读取单个结果（如果不存在返回 default）。"""
    arm_key = _arm_key_for_obj(obj_arm)
    return _psd_results_cache.get(arm_key, {}).get(key, default)

def psd_clear_results_for_arm(obj_arm):
    """清除某个 arm 的缓存结果。"""
    arm_key = _arm_key_for_obj(obj_arm)
    _psd_results_cache.pop(arm_key, None)

def psd_clear_all_results():
    """清除所有缓存（全局）。"""
    _psd_results_cache.clear()

#=====================================================


def _safe_name(s: str) -> str:
    s = (s or "").strip()
    s = re.sub(r"\s+", "_", s)
    s = re.sub(r"[^0-9A-Za-z_\-]", "_", s)
    return s

def _get_selected_pair_bone(context):
    """从活动骨架的psd_bone_pairs中返回当前选定的骨骼名称，如果没有则返回'<NONE>'。"""
    arm = context.object
    if not arm or arm.type != 'ARMATURE':
        return '<NONE>'
    try:
        idx = int(getattr(arm, 'psd_bone_pairs_index', 0))
        if idx < 0 or idx >= len(arm.psd_bone_pairs):
            return '<NONE>'
        bn = arm.psd_bone_pairs[idx].bone_name
        if not bn:
            return '<NONE>'
        return bn
    except Exception:
        return '<NONE>'

def _get_rest_matrix(arm_obj, bone_name):
    bone = arm_obj.data.bones.get(bone_name)
    if not bone:
        raise RuntimeError(f"在骨架数据 '{arm_obj.name}' 上找不到静止姿态的骨骼 '{bone_name}'")
    return bone.matrix_local.copy()

def _get_pose_matrix(obj_with_pose, bone_name):
    pb = obj_with_pose.pose.bones.get(bone_name)
    if not pb:
        raise RuntimeError(f"在对象 '{obj_with_pose.name}' 上找不到姿态骨骼 '{bone_name}'")
    return pb.matrix.copy()

def _capture_bone_local_rotation_deg(arm_obj, bone_name, depsgraph=None):
    """
    返回骨骼当前 pose 相对于其 rest pose 的局部旋转（度），作为 (x,y,z)（以 XYZ 欧拉返回）。
    """
    if not arm_obj or arm_obj.type != 'ARMATURE':
        raise RuntimeError("必须传递一个骨架对象")

    source_obj = arm_obj
    if depsgraph is not None:
        try:
            eval_obj = arm_obj.evaluated_get(depsgraph)
            if eval_obj:
                source_obj = eval_obj
        except Exception:
            source_obj = arm_obj

    rest_mat = _get_rest_matrix(arm_obj, bone_name)
    pose_mat = _get_pose_matrix(source_obj, bone_name)

    rest_bone = arm_obj.data.bones.get(bone_name)
    parent_name = rest_bone.parent.name if rest_bone and rest_bone.parent else None

    if parent_name:
        parent_rest = arm_obj.data.bones[parent_name].matrix_local.copy()
        parent_pose = _get_pose_matrix(source_obj, parent_name)
        try:
            rest_local = parent_rest.inverted_safe() @ rest_mat
        except Exception:
            rest_local = rest_mat.copy()
        try:
            pose_local = parent_pose.inverted_safe() @ pose_mat
        except Exception:
            pose_local = pose_mat.copy()
    else:
        rest_local = rest_mat
        pose_local = pose_mat

    try:
        delta = rest_local.inverted_safe() @ pose_local
    except Exception:
        delta = pose_local.copy()

    delta_rot = delta.to_3x3().to_euler('XYZ')
    return (math.degrees(delta_rot.x), math.degrees(delta_rot.y), math.degrees(delta_rot.z))

def _capture_bone_local_translation(arm_obj, bone_name, depsgraph=None):
    """
    捕获骨骼在父空间下的相对位移 (x,y,z)。Rest始终为(0,0,0)，Pose为poseBone.location。
    """
    if not arm_obj or arm_obj.type != 'ARMATURE':
        raise RuntimeError("必须传递一个骨架对象")

    source_obj = arm_obj
    if depsgraph is not None:
        try:
            eval_obj = arm_obj.evaluated_get(depsgraph)
            if eval_obj:
                source_obj = eval_obj
        except Exception:
            source_obj = arm_obj

    pb = source_obj.pose.bones.get(bone_name)
    if not pb:
        raise RuntimeError(f"在对象 '{source_obj.name}' 上找不到姿态骨骼 '{bone_name}'")

    rest_loc = pb.location.copy()
    pose_loc = pb.location.copy()
    return (rest_loc, pose_loc)

def _capture_bone_local_translation_effective(arm_obj, bone_name, depsgraph=None):
    """
    返回骨骼当前 pose 相对于其 rest pose 的等效局部平移（包含所有约束影响）。
    返回 Vector((x, y, z))，单位与 Blender 一致。
    与 _capture_bone_local_rotation_deg 逻辑完全一致，支持世界空间约束。
    """
    if not arm_obj or arm_obj.type != 'ARMATURE':
        raise RuntimeError("必须传递一个骨架对象")

    source_obj = arm_obj
    if depsgraph is not None:
        try:
            eval_obj = arm_obj.evaluated_get(depsgraph)
            if eval_obj:
                source_obj = eval_obj
        except Exception:
            source_obj = arm_obj

    rest_mat = _get_rest_matrix(arm_obj, bone_name)
    pose_mat = _get_pose_matrix(source_obj, bone_name)

    rest_bone = arm_obj.data.bones.get(bone_name)
    parent_name = rest_bone.parent.name if rest_bone and rest_bone.parent else None

    if parent_name:
        parent_rest = arm_obj.data.bones[parent_name].matrix_local.copy()
        parent_pose = _get_pose_matrix(source_obj, parent_name)
        try:
            rest_local = parent_rest.inverted_safe() @ rest_mat
        except Exception:
            rest_local = rest_mat.copy()
        try:
            pose_local = parent_pose.inverted_safe() @ pose_mat
        except Exception:
            pose_local = pose_mat.copy()
    else:
        rest_local = rest_mat
        pose_local = pose_mat

    try:
        delta = rest_local.inverted_safe() @ pose_local
    except Exception:
        delta = pose_local.copy()

    return delta.to_translation()


def _capture_bone_local_scale(arm_obj, bone_name, depsgraph=None):
    """
    捕获骨骼在父空间下的相对缩放 (x,y,z)。Rest 和 Pose 都为 poseBone.scale。
    """
    if not arm_obj or arm_obj.type != 'ARMATURE':
        raise RuntimeError("必须传递一个骨架对象")

    source_obj = arm_obj
    if depsgraph is not None:
        try:
            eval_obj = arm_obj.evaluated_get(depsgraph)
            if eval_obj:
                source_obj = eval_obj
        except Exception:
            source_obj = arm_obj

    pb = source_obj.pose.bones.get(bone_name)
    if not pb:
        raise RuntimeError(f"在对象 '{source_obj.name}' 上找不到姿态骨骼 '{bone_name}'")

    rest_sca = pb.scale.copy()  # 假设 rest 是默认 (1,1,1)，但实际捕捉当前作为 rest
    pose_sca = pb.scale.copy()
    return (rest_sca, pose_sca)

def psd_set_result_datablock_only(obj_arm, key, value, verbose=False):
    try:
        fw = float(value)
    except Exception:
        return False

    wrote = False
    try:
        arm_db = bpy.data.armatures.get(obj_arm.data.name)
        if arm_db is not None:
            prev = arm_db.get(key, None)
            if prev is None or abs(prev - fw) > 1e-6:
                arm_db[key] = fw
                wrote = True
                if verbose:
                    print(f"[PSD] 已写入骨架数据块 '{arm_db.name}': {key} = {fw}")
            try:
                arm_db.update_tag()
            except Exception:
                pass
    except Exception as e:
        if verbose:
            print("psd_set_result_datablock_only: 写入失败:", e)
        return False
    return wrote


def bone_items(self, context):
    obj = context.object
    items = []
    if obj and obj.type == 'ARMATURE':
        for i, pb in enumerate(obj.pose.bones):
            items.append((pb.name, pb.name, "", i))
    if not items:
        items = [("<NONE>", "<无骨骼>", "", 0)]
    return items

# 其他辅助，如 _is_animation_playing（如果存在，原脚本中提到）
def _is_animation_playing():
    try:
        scr = bpy.context.screen
        return bool(getattr(scr, "is_animation_playing", False))
    except Exception:
        try:
            return bool(bpy.context.scene and getattr(bpy.context.scene, "is_playing", False))
        except Exception:
            return False
        
