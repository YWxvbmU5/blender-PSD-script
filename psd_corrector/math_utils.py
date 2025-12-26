import math
from mathutils import Vector, Euler, Quaternion

def _euler_deg_to_dir(rot_deg, axis='Z'):
    # 原脚本中 _euler_deg_to_dir 函数完整复制
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
    # 原脚本中 _euler_deg_to_quat 函数完整复制
    ex, ey, ez = rot_deg
    return Euler((math.radians(ex), math.radians(ey), math.radians(ez)), 'XYZ').to_quaternion()

def _swing_twist_decompose(q: Quaternion, twist_axis: Vector):
    # 原脚本中 _swing_twist_decompose 函数完整复制
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
    # 原脚本中 _signed_angle_from_quat 函数完整复制
    q = q.copy()
    if q.w < 0:
        q.negate()
    theta = 2.0 * math.acos(q.w)
    vec = Vector((q.x, q.y, q.z))
    sign = 1.0 if vec.dot(axis.normalized()) >= 0 else -1.0
    return sign * math.degrees(theta)

def _triangular_ratio(cur, target):
    # 原脚本中 _triangular_ratio 函数完整复制
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