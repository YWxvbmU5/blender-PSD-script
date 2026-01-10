import math
from mathutils import Vector, Euler, Quaternion

def _euler_deg_to_dir(rot_deg, axis='Z'):
    
    ex, ey, ez = rot_deg
    e = Euler((math.radians(ex), math.radians(ey), math.radians(ez)), 'XYZ')
    mat = e.to_matrix()
    if axis == 'X':
        base = Vector((1.0, 0.0, 0.0))
    elif axis == 'Y':
        base = Vector((0.0, 1.0, 0.0))
    else:
        base = Vector((0.0, 0.0, 1.0))
    dir_vec = mat @ base
    if dir_vec.length == 0:
        return Vector((0.0, 0.0, 1.0))
    return dir_vec.normalized()

def _euler_deg_to_quat(rot_deg):
    
    ex, ey, ez = rot_deg
    return Euler((math.radians(ex), math.radians(ey), math.radians(ez)), 'XYZ').to_quaternion()

def _swing_twist_decompose(q: Quaternion, twist_axis: Vector):
    
    if twist_axis.length == 0:
        return q.copy(), Quaternion((1.0, 0.0, 0.0, 0.0))
    axis = twist_axis.normalized()
    q_vec = Vector((q.x, q.y, q.z))
    proj = axis * q_vec.dot(axis)
    twist = Quaternion((q.w, proj.x, proj.y, proj.z))
    try:
        twist.normalize()
    except Exception:
        twist = Quaternion((1.0, 0.0, 0.0, 0.0))
    swing = q @ twist.inverted()
    return swing, twist

def _signed_angle_from_quat(q: Quaternion, axis: Vector):
    
    q = q.copy()
    if q.w < 0:
        q.negate()
    theta = 2.0 * math.acos(q.w)
    vec = Vector((q.x, q.y, q.z))
    sign = 1.0 if vec.dot(axis.normalized()) >= 0 else -1.0
    return sign * math.degrees(theta)

def _triangular_ratio(cur, target):
    
    if target is None:
        return 0.0
    if abs(target) < 1e-6:
        return 1.0 if abs(cur) < 1e-3 else 0.0
    r = float(cur) / float(target)
    if r < 0.0:
        return 0.0
    if r > 2.0:
        return 0.0
    if r > 1.0:
        return 2.0 - r
    return r

#==========权重算法================================
def compute_direct_channel_weight(
    entry, bone_to_cur_rot, arm, bn,
    psd_set_result_cache_only, PREFIX_RESULT, _safe_name
):
    """计算 Direct Channel 权重并写入缓存"""
    try:
        key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(entry.name)}"
        cur_rot = bone_to_cur_rot.get(bn)

        if cur_rot is None:
            psd_set_result_cache_only(arm, key_rot, 0.0, verbose=False)
            return

        mode = getattr(entry, 'record_rot_channel_mode', 'NONE')
        ch_axis = getattr(entry, 'channel_axis', 'X')
        axis_idx_map = {'X': 0, 'Y': 1, 'Z': 2}
        axis_idx = axis_idx_map.get(ch_axis, 0)
        cur_rot_value = float(cur_rot[axis_idx])
        q_cur = _euler_deg_to_quat(tuple(cur_rot))

        w_deg = cur_rot_value

        if mode in ('record_rot_SWING_X_TWIST', 'record_rot_SWING_Y_TWIST', 'record_rot_SWING_Z_TWIST'):
            if mode == 'record_rot_SWING_X_TWIST':
                twist_axis = Vector((1.0, 0.0, 0.0))
            elif mode == 'record_rot_SWING_Y_TWIST':
                twist_axis = Vector((0.0, 1.0, 0.0))
            else:
                twist_axis = Vector((0.0, 0.0, 1.0))

            swing_q, twist_q = _swing_twist_decompose(q_cur, twist_axis)

            def _clamp1(v):
                if v is None:
                    return 0.0
                v = float(v)
                return max(-1.0, min(1.0, v))

            if ch_axis == 'X':
                sx = _clamp1(swing_q.x)
                w_deg = math.degrees(2.0 * math.asin(sx))
            elif ch_axis == 'Y':
                w_deg = _signed_angle_from_quat(twist_q, twist_axis)
            elif ch_axis == 'Z':
                sz = _clamp1(swing_q.z)
                w_deg = math.degrees(2.0 * math.asin(sz))

        w = math.radians(w_deg)
        if math.isnan(w):
            w = 0.0
        psd_set_result_cache_only(arm, key_rot, w, verbose=False)

    except Exception as e:
        print(f"[Direct Channel] 计算出错 {entry.name}: {e}")
        key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(entry.name)}"
        psd_set_result_cache_only(arm, key_rot, 0.0, verbose=False)


def compute_rotation_weight(
    entry, bone_to_cur_rot, bn, arm,
    psd_set_result_cache_only, PREFIX_RESULT, _safe_name
):
    """计算 Rotation Channel 权重并写入缓存"""
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
            #不要开启这个，基本没什么用。:P
            # elif getattr(entry, 'rot_channel_mode', 'NONE') != 'NONE':
            #     axis_idx_map = {'SWING_X_TWIST': 0, 'SWING_Y_TWIST': 1, 'SWING_Z_TWIST': 2}
            #     axis_idx = axis_idx_map.get(entry.rot_channel_mode, 2)
            #     target_twist_angle = float(entry.pose_rot[axis_idx] - entry.rest_rot[axis_idx])
            #     cur_twist_angle = float(cur_rot[axis_idx] - entry.rest_rot[axis_idx])
            #     w_twist = _triangular_ratio(cur_twist_angle, target_twist_angle)
            #     w = float(max(0.0, min(1.0, w_twist)))
            else:
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
            if math.isnan(w):
                w = 0.0
            w = max(0.0, min(1.0, w))
        else:
            w = 0.0

        key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(entry.name)}"
        psd_set_result_cache_only(arm, key_rot, w, verbose=False)

    except Exception as e:
        print(f"[Rotation Channel] 计算出错 {entry.name}: {e}")


def compute_location_weight(
    entry, bone_to_cur_loc, bn, arm,
    psd_set_result_cache_only, PREFIX_RESULT_LOC, _safe_name
):
    """计算 Location Channel 权重并写入缓存"""
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
                rest_vec = Vector(getattr(entry, 'rest_loc', (0.0, 0.0, 0.0)))
                pose_vec = Vector(entry.pose_loc)
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
            if math.isnan(w_loc):
                w_loc = 0.0
            w_loc = max(0.0, min(1.0, w_loc))
        else:
            w_loc = 0.0

        key_loc = f"{PREFIX_RESULT_LOC}{_safe_name(bn)}_{_safe_name(entry.name)}"
        psd_set_result_cache_only(arm, key_loc, w_loc, verbose=False)

    except Exception as e:
        print(f"[Location Channel] 计算出错 {entry.name}: {e}")


def compute_scale_weight(
    entry, bone_to_cur_sca, bn, arm,
    psd_set_result_cache_only, PREFIX_RESULT_SCA, _safe_name
):
    """计算 Scale Channel 权重并写入缓存"""
    try:
        cur_sca = bone_to_cur_sca.get(bn)
        if cur_sca is not None:
            rest_vec = Vector(getattr(entry, 'rest_sca', (1.0, 1.0, 1.0)))
            pose_vec = Vector(entry.pose_sca)
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
        else:
            w_sca = 0.0

        key_sca = f"{PREFIX_RESULT_SCA}{_safe_name(bn)}_{_safe_name(entry.name)}"
        psd_set_result_cache_only(arm, key_sca, w_sca, verbose=False)

    except Exception as e:
        print(f"[Scale Channel] 计算出错 {entry.name}: {e}")


def compute_triggers(
    orig_arm,
    psd_set_result_cache_only, PREFIX_RESULT_LOC, _safe_name
):
    """计算所有 Triggers 并写入缓存"""
    if not getattr(orig_arm, "psd_triggers", None):
        return

    for trig in orig_arm.psd_triggers:
        if not trig.enabled:
            continue
        trig.last_weight = 0.0
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
        psd_set_result_cache_only(orig_arm, f"{key_base}_w", float(w), verbose=False)