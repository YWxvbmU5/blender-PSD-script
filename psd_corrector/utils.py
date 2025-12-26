import bpy
import re
import math
from mathutils import Vector, Euler, Quaternion  # 如果需要

PREFIX_RESULT = "psd_rot_"  
PREFIX_RESULT_LOC = "psd_loc_"  
PREFIX_RESULT_SCA = "psd_sca_"  

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
        
