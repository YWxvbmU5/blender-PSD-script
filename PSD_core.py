bl_info = {
    "name": "PSD校正器",
    "author": "s0lus",
    "version": (2, 8),
    "blender": (4, 3, 2),
    "location": "3D视图 > 侧边栏 > PSD",
    "description": "捕捉骨骼的静止/姿态旋转和位置，并将其另存为逐骨架的已保存姿态。PSD算法为每个已保存的姿态（旋转+可选位移）计算单个浮点值，并将其写入骨架数据中。",
    "category": "动画",
}


import bpy
import math
import re
import time
from mathutils import Vector, Euler, Quaternion
import json
import os
from bpy_extras.io_utils import ImportHelper, ExportHelper
from bpy.props import StringProperty, BoolProperty, FloatProperty, CollectionProperty, IntProperty, EnumProperty


PREFIX_RESULT = "psd_rot_"  # 旋转结果前缀
PREFIX_RESULT_LOC = "psd_loc_"  # 位移结果前缀

# 全局计算节流，防止在短时间内重复调用
last_compute_time = 0.0

# -------------------------------
# 性能存储 (内存中, 插件生命周期内有效)
# -------------------------------
_psd_perf_stats = {}

# -------------------------------
# 辅助函数
# -------------------------------
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

def bone_items(self, context):
    obj = context.object
    items = []
    if obj and obj.type == 'ARMATURE':
        for i, pb in enumerate(obj.pose.bones):
            items.append((pb.name, pb.name, "", i))
    if not items:
        items = [("<NONE>", "<无骨骼>", "", 0)]
    return items

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

# ---------- 新增：欧拉度 -> 四元数, 以及 swing-twist 分解工具 ----------

def _euler_deg_to_quat(rot_deg):
    """把以度为单位的欧拉 (X,Y,Z) 转成 Quaternion (使用 'XYZ' 顺序)。"""
    ex, ey, ez = rot_deg
    return Euler((math.radians(ex), math.radians(ey), math.radians(ez)), 'XYZ').to_quaternion()

def _swing_twist_decompose(q: Quaternion, twist_axis: Vector):
    """把 q 分解为 swing 和 twist，其中 swing 是将 twist_axis 定向到其最终方向的最短旋转，然后 twist 是围绕该轴的剩余旋转。
    返回 (swing_quat, twist_quat)，顺序为 q = twist @ swing (先 swing 后 twist)。
    """
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
    """沿用原先逻辑的“三角包络”方式：
    ratio = cur/target
    if target~0 则特殊处理（匹配/不匹配）
    否则：0 <= ratio <= 2 -> 当 ratio in [0,1] 线性增, ratio in (1,2] 对称下降, ratio >2 或 <0 -> 0
    返回 [0,1]
    注意：cur 和 target 都以同一单位（例如度）传入。
    """
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

# -----------------------------------
# PSD 配置导出 / 导入
# -----------------------------------

class PSDExportConfig(bpy.types.Operator, ExportHelper):
    """导出当前骨架的 PSD 配置（包含每个 bone_pair 对应的所有条目）"""
    bl_idname = "psd.export_config"
    bl_label = "导出 PSD 配置"
    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架对象")
            return {'CANCELLED'}

        data = {"bone_pairs": [], "saved_by_bone": {}, "orphan_saved_poses": []}

        # 导出 bone_pairs
        try:
            data["bone_pairs"] = [p.bone_name for p in arm.psd_bone_pairs]
        except:
            pass

        # 导出 saved_poses 并按 bone_name 分组
        try:
            for e in arm.psd_saved_poses:
                item = {
                    "name": e.name,
                    "bone_name": e.bone_name,
                    "rest_rot": list(e.rest_rot),
                    "pose_rot": list(e.pose_rot),
                    "has_rot": bool(e.has_rot),
                    "rot_channel_mode": getattr(e, "rot_channel_mode", "NONE"),
                    "cone_enabled": bool(e.cone_enabled),
                    "cone_angle": float(e.cone_angle),
                    "cone_axis": getattr(e, "cone_axis", "Z"),
                    "rest_loc": list(e.rest_loc),
                    "pose_loc": list(e.pose_loc),
                    "has_loc": bool(e.has_loc),
                    "loc_enabled": bool(e.loc_enabled),
                    "loc_radius": float(e.loc_radius),
                    "group_name": getattr(e, "group_name", "")
                }
                if e.bone_name in data["bone_pairs"]:
                    data["saved_by_bone"].setdefault(e.bone_name, []).append(item)
                else:
                    data["orphan_saved_poses"].append(item)
        except Exception as err:
            self.report({'WARNING'}, f"导出条目时出错: {err}")

        # 写文件
        try:
            with open(self.filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self.report({'INFO'}, f"已导出到 {self.filepath}")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"写入失败: {e}")
            return {'CANCELLED'}

    def invoke(self, context, event):
        arm = context.object
        if arm and arm.type == 'ARMATURE':
            self.filepath = bpy.path.ensure_ext(arm.name + "_psd_config.json", ".json")
        return super().invoke(context, event)


class PSDImportConfig(bpy.types.Operator, ImportHelper):
    """从 JSON 导入 PSD 配置（合并并在遇重名时跳过，不覆盖已有条目）"""
    bl_idname = "psd.import_config"
    bl_label = "导入 PSD 配置（合并-跳过重名）"
    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json", options={'HIDDEN'})

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架对象（Armature）再导入")
            return {'CANCELLED'}

        try:
            with open(self.filepath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            self.report({'ERROR'}, f"读取配置文件失败: {e}")
            return {'CANCELLED'}

        # ---------- 合并 bone_pairs（不删除已有） ----------
        existing_pairs = {p.bone_name for p in arm.psd_bone_pairs}
        for bn in data.get("bone_pairs", []):
            if bn not in existing_pairs:
                p = arm.psd_bone_pairs.add()
                p.bone_name = str(bn or "")
                existing_pairs.add(bn)

        # ---------- 准备导入条目列表 ----------
        entries = []
        for bone, ents in data.get("saved_by_bone", {}).items():
            for ent in ents:
                entries.append(ent)
        for ent in data.get("orphan_saved_poses", []):
            entries.append(ent)

        # ---------- 现有条目集合（用于检测同 bone_name + name 冲突） ----------
        existing_entries = {(e.bone_name, e.name) for e in arm.psd_saved_poses}

        added = 0
        skipped = 0

        # ---------- 导入每个条目（遇重名跳过） ----------
        for ent in entries:
            bone_name = str(ent.get("bone_name", "") or "")
            base_name = str(ent.get("name", "entry") or "entry")

            key = (bone_name, base_name)
            if key in existing_entries:
                skipped += 1
                continue  # 跳过已有同名条目

            # 新增条目
            new = arm.psd_saved_poses.add()
            # 安全赋值（逐项赋值以避免类型错误）
            try:
                new.name = base_name
            except Exception:
                pass
            try:
                new.bone_name = bone_name
            except Exception:
                pass

            try:
                rr = ent.get("rest_rot", [0.0, 0.0, 0.0])
                if len(rr) >= 3:
                    new.rest_rot = (float(rr[0]), float(rr[1]), float(rr[2]))
            except Exception:
                pass
            try:
                pr = ent.get("pose_rot", [0.0, 0.0, 0.0])
                if len(pr) >= 3:
                    new.pose_rot = (float(pr[0]), float(pr[1]), float(pr[2]))
            except Exception:
                pass
            try:
                new.has_rot = bool(ent.get("has_rot", False))
            except Exception:
                pass
            try:
                if hasattr(new, "rot_channel_mode"):
                    new.rot_channel_mode = str(ent.get("rot_channel_mode", getattr(new, "rot_channel_mode", "NONE") or "NONE"))
            except Exception:
                pass
            try:
                new.cone_enabled = bool(ent.get("cone_enabled", False))
                new.cone_angle = float(ent.get("cone_angle", getattr(new, "cone_angle", 60.0)))
                if hasattr(new, "cone_axis"):
                    new.cone_axis = str(ent.get("cone_axis", getattr(new, "cone_axis", "Z") or "Z"))
            except Exception:
                pass

            # 位置相关
            try:
                rl = ent.get("rest_loc", [0.0, 0.0, 0.0])
                if len(rl) >= 3:
                    new.rest_loc = (float(rl[0]), float(rl[1]), float(rl[2]))
            except Exception:
                pass
            try:
                pl = ent.get("pose_loc", [0.0, 0.0, 0.0])
                if len(pl) >= 3:
                    new.pose_loc = (float(pl[0]), float(pl[1]), float(pl[2]))
            except Exception:
                pass
            try:
                new.has_loc = bool(ent.get("has_loc", False))
                new.loc_enabled = bool(ent.get("loc_enabled", False))
                new.loc_radius = float(ent.get("loc_radius", getattr(new, "loc_radius", 0.1)))
            except Exception:
                pass

            # group_name 或其他字符串字段
            try:
                if hasattr(new, "group_name"):
                    new.group_name = str(ent.get("group_name", getattr(new, "group_name", "") or ""))
            except Exception:
                pass

            # 额外字段：保守尝试写入（跳过已处理字段）
            for k, v in ent.items():
                if k in {"name","bone_name","rest_rot","pose_rot","has_rot","rot_channel_mode",
                         "cone_enabled","cone_angle","cone_axis","rest_loc","pose_loc",
                         "has_loc","loc_enabled","loc_radius","group_name"}:
                    continue
                try:
                    if hasattr(new, k):
                        setattr(new, k, v)
                except Exception:
                    pass

            # 标记为已存在，避免同一导入文件中重复导入相同条目
            existing_entries.add(key)
            added += 1

        # 可选：将索引指向最后一个新添加的条目
        if len(arm.psd_saved_poses) > 0:
            arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1

        self.report({'INFO'}, f"导入完成：新增 {added} 条，跳过 {skipped} 条重复条目（同名同骨骼）")
        return {'FINISHED'}


# -------------------------------
# 已保存姿态条目 (逐骨架集合)
# -------------------------------
class PSDSavedPose(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="条目名称", default="default")
    bone_name: bpy.props.StringProperty(name="骨骼")
    # 旋转通道
    record_rot_channel_mode: bpy.props.EnumProperty(
    name="旋转通道模式",
    description="使用摇摆+扭转分解来计算权重（若非 'NONE' 且未启用锥形衰减时生效）",
    items=[
        ('NONE', "默认", "不使用通道模式 (保持原先的向量投影法)") ,
        ('record_rot_SWING_X_TWIST', "摆动和 X 扭转", "将 X 轴作为扭转轴"),
        ('record_rot_SWING_Y_TWIST', "摆动和 Y 扭转", "将 Y 轴作为扭转轴"),
        ('record_rot_SWING_Z_TWIST', "摆动和 Z 扭转", "将 Z 轴作为扭转轴"),
    ],
    default='NONE'
    )
    rest_rot: bpy.props.FloatVectorProperty(name="静止旋转 (度)", size=3, default=(0.0,0.0,0.0))
    pose_rot: bpy.props.FloatVectorProperty(name="姿态旋转 (度)", size=3, default=(0.0,0.0,0.0))
    has_rot: bpy.props.BoolProperty(name="包含旋转", default=False)
    # 新增：旋转通道模式（参考 Blender 驱动器的 "旋转通道模式"）
    rot_channel_mode: bpy.props.EnumProperty(
        name="旋转通道模式",
        description="使用摇摆+扭转分解来计算权重（若非 'NONE' 且未启用锥形衰减时生效）",
        items=[
            ('NONE', "默认", "不使用通道模式 (保持原先的向量投影法)") ,
            ('SWING_X_TWIST', "only X 扭转", "将 X 轴作为扭转轴"),
            ('SWING_Y_TWIST', "only Y 扭转", "将 Y 轴作为扭转轴"),
            ('SWING_Z_TWIST', "only Z 扭转", "将 Z 轴作为扭转轴"),
        ],
        default='NONE'
    )

    # 旋转的锥形衰减 (现有行为)
    cone_enabled: bpy.props.BoolProperty(name="锥形衰减", default=False)
    cone_angle: bpy.props.FloatProperty(name="锥角 (度)", default=60.0, min=0.0, max=180.0)
    cone_axis: bpy.props.EnumProperty(
        name="锥体轴向",
        description="用作锥体中心方向的局部轴",
        items=[('X','X',''), ('Y','Y',''), ('Z','Z','')],
        default='Z'
    )

    # 位移通道
    rest_loc: bpy.props.FloatVectorProperty(name="静止位置", size=3, default=(0.0,0.0,0.0))
    pose_loc: bpy.props.FloatVectorProperty(name="姿态位置", size=3, default=(0.0,0.0,0.0))
    has_loc: bpy.props.BoolProperty(name="包含位移", default=False)
    loc_enabled: bpy.props.BoolProperty(name="启用位移衰减", default=False)
    loc_radius: bpy.props.FloatProperty(name="位移半径", default=0.1, min=0.0, soft_max=10.0)

    is_direct_channel: bpy.props.BoolProperty(name="Direct Channel Record", default=False)
    channel_axis: bpy.props.EnumProperty(
        name="Channel Axis",
        items=[('X', "X", ""), ('Y', "Y", ""), ('Z', "Z", "")],
        default='X'
    )


# -------------------------------
# TriggerUIList
# -------------------------------

class PSDBoneTriggerUIList(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        # item 是 PSDBoneTrigger
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            row = layout.row(align=True)
            row.prop(item, "enabled", text="")
            row.label(text=item.name)
            row.label(text=f"[T:{item.target_bone} R:{item.radius:.2f}]")
        else:
            layout.label(text=item.name)



# -------------------------------
# 骨骼对 (过滤器) - 逐骨架
# -------------------------------
class PSDBonePair(bpy.types.PropertyGroup):
    bone_name: bpy.props.StringProperty(name="骨骼", default="")

class PSDBonePairUIList(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        arm = context.object
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            row = layout.row(align=True)
            if arm and hasattr(arm, "data"):
                row.prop_search(item, "bone_name", arm.data, "bones", text="")
            else:
                row.prop(item, "bone_name", text="")
            count = 0
            try:
                if arm and hasattr(arm, "psd_saved_poses"):
                    count = sum(1 for e in arm.psd_saved_poses if e.bone_name == item.bone_name)
            except Exception:
                count = 0
            row.label(text=f"{count}", icon='DOT')
        elif self.layout_type == 'GRID':
            layout.alignment = 'CENTER'
            layout.label(text="")

class PSD_OT_AddBonePair(bpy.types.Operator):
    bl_idname = "psd.add_bone_pair"
    bl_label = "添加骨骼对"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        existing = set(p.bone_name for p in arm.psd_bone_pairs if p.bone_name)
        chosen = None
        for b in arm.data.bones:
            if b.name not in existing:
                chosen = b.name
                break
        if chosen is None:
            chosen = arm.data.bones[0].name if len(arm.data.bones) > 0 else ""
        pair = arm.psd_bone_pairs.add()
        pair.bone_name = chosen
        arm.psd_bone_pairs_index = len(arm.psd_bone_pairs) - 1
        return {'FINISHED'}

class PSD_OT_RemoveBonePair(bpy.types.Operator):
    bl_idname = "psd.remove_bone_pair"
    bl_label = "移除骨骼对"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        idx = arm.psd_bone_pairs_index
        if arm.psd_bone_pairs:
            arm.psd_bone_pairs.remove(idx)
            arm.psd_bone_pairs_index = max(0, idx-1)
        return {'FINISHED'}

class PSD_OT_MoveBonePair(bpy.types.Operator):
    bl_idname = "psd.move_bone_pair"
    bl_label = "移动骨骼对"
    direction: bpy.props.EnumProperty(items=[('UP','上',''), ('DOWN','下','')], default='UP')

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            return {'CANCELLED'}
        idx = arm.psd_bone_pairs_index
        if idx < 0 or idx >= len(arm.psd_bone_pairs):
            return {'CANCELLED'}
        if self.direction == 'UP' and idx > 0:
            try:
                arm.psd_bone_pairs.move(idx, idx-1)
                arm.psd_bone_pairs_index = idx-1
            except Exception:
                pass
        elif self.direction == 'DOWN' and idx < len(arm.psd_bone_pairs)-1:
            try:
                arm.psd_bone_pairs.move(idx, idx+1)
                arm.psd_bone_pairs_index = idx+1
            except Exception:
                pass
        return {'FINISHED'}

# -------------------------------
# PSD 计算 (由处理器使用)
# -------------------------------
def _psd_compute_all(depsgraph=None):
    """
    计算所有 armature 的 PSD 结果。
    对于每个 saved entry，会分别计算 rotation 权重（如原先）与 location 权重（轴向分离线性或球体衰减），
    并分别写入不同的 datablock key（psd_result_... 与 psd_loc_...）。
    此处在旋转通道部分新增了基于四元数的摇摆-扭转分解（当条目选择了 rot_channel_mode 且未启用 cone_enabled 时）。
    """
    global last_compute_time, _psd_perf_stats
    current_time = time.time()

    # 计算最小间隔：播放时不节流 -> min_interval = 0.0；空闲时读取场景属性 psd_idle_hz
    try:
        sc = bpy.context.scene if (bpy.context and getattr(bpy.context, "scene", None)) else None
        if _is_animation_playing():
            # 播放时不进行节流（每帧/每次 handler 调用都会运行）
            min_interval = 0.0
        else:
            hz = int(getattr(sc, "psd_idle_hz", 10) or 10)
            # 限制到合理范围，避免零除或极端值
            if hz < 1:
                hz = 1
            elif hz > 240:
                hz = 240
            min_interval = 1.0 / float(hz)
    except Exception:
        # 回退到原先的 20Hz（0.05s）
        min_interval = 0.05

    # 只有在 min_interval > 0 时才做节流判断
    if min_interval > 0.0:
        if current_time - last_compute_time < min_interval:
            return
    # 更新上次计算时间（即算过一次就记下时间）
    last_compute_time = current_time



    # 确定是否收集性能统计信息
    perf_enabled = False
    try:
        sc = bpy.context.scene if bpy.context and bpy.context.scene else None
        perf_enabled = bool(sc and getattr(sc, 'psd_perf_enabled', False))
        history_len = int(getattr(sc, 'psd_perf_history_len', 10) or 10)
    except Exception:
        perf_enabled = False
        history_len = 10

    t_start_total = time.perf_counter() if perf_enabled else None

    for arm in [o for o in bpy.data.objects if o.type == 'ARMATURE']:
        try:
            saved = getattr(arm, 'psd_saved_poses', None)
            if not saved:
                continue

            # 从arm.psd_bone_pairs构建筛选集合（如果有）
            bone_filter = None
            try:
                if getattr(arm, "psd_bone_pairs", None) and len(arm.psd_bone_pairs) > 0:
                    bone_filter = set([p.bone_name for p in arm.psd_bone_pairs if p.bone_name])
            except Exception:
                bone_filter = None

            if bone_filter is None:
                bone_filter = set(e.bone_name for e in saved if e.bone_name)

            # 预先计算所有相关骨骼的当前旋转和位移（避免重复捕捉）
            bone_to_cur_rot = {}
            bone_to_cur_loc = {}
            source_obj = arm
            if depsgraph is not None:
                try:
                    eval_obj = arm.evaluated_get(depsgraph)
                    if eval_obj:
                        source_obj = eval_obj
                except Exception:
                    source_obj = arm
            for bn in bone_filter:
                try:
                    # 旋转 (度)
                    cur_deg = _capture_bone_local_rotation_deg(arm, bn, depsgraph=depsgraph)
                    bone_to_cur_rot[bn] = Vector(cur_deg)
                except Exception:
                    pass
                try:
                    # 位移：获取当前相对姿态位置 (pb.location)
                    pb = source_obj.pose.bones.get(bn)
                    if pb:
                        bone_to_cur_loc[bn] = pb.location.copy()
                except Exception:
                    pass

            # 为此骨架准备性能统计容器（如果启用）
            arm_stats = None
            if perf_enabled:
                arm_stats = _psd_perf_stats.setdefault(arm.name, {"last_total_ms": 0.0, "last_arm_ms": 0.0, "entries": {}})

            t_start_arm = time.perf_counter() if perf_enabled else None
            for entry in saved:
                bn = entry.bone_name
                en = entry.name
                if not bn or not en:
                    continue
                if bn not in bone_filter:
                    continue
                # 如果骨骼未被采样，则跳过
                if bn not in bone_to_cur_rot and bn not in bone_to_cur_loc:
                    continue

                # 单个条目性能计时开始
                t_entry_start = time.perf_counter() if perf_enabled else None

                # Direct channel record (if enabled)

                if getattr(entry, 'is_direct_channel', False):
                    try:
                        key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                        cur_rot = bone_to_cur_rot.get(bn)  # 获取当前旋转

                        # 如果当前旋转没有获取到值，则直接写 0
                        if cur_rot is None:
                            print(f"警告：没有获取到骨骼 {bn} 的旋转数据，写入 0")
                            psd_set_result_datablock_only(arm, key_rot, 0.0, verbose=False)
                        else:
                            # 获取记录旋转通道模式
                            mode = getattr(entry, 'record_rot_channel_mode', 'NONE')

                            # 获取当前旋转通道的值
                            axis_idx_map = {'X': 0, 'Y': 1, 'Z': 2}
                            ch_axis = getattr(entry, 'channel_axis', 'X')  # 获取通道轴
                            axis_idx = axis_idx_map.get(ch_axis, 0)  # 获取对应的轴索引
                            cur_rot_value = cur_rot[axis_idx]

                            # 获取当前旋转的四元数表示
                            q_cur = _euler_deg_to_quat(tuple(cur_rot))

                            # --- 计算旋转分解：摆动（Swing）和扭转（Twist） ---
                            if mode == 'record_rot_SWING_Y_TWIST':
                                twist_axis = Vector((0.0, 1.0, 0.0))  # Y 轴为扭转轴
                                swing_t, twist_t = _swing_twist_decompose(q_cur, twist_axis)

                                # 计算 W 旋转（摆动角度）
                                W_cur = math.degrees(2.0 * math.acos(_clamp01(swing_t.w)))
                                print(f"摆动角度 W_cur: {W_cur}")

                                # 计算扭转角度
                                cur_twist_angle = _signed_angle_from_quat(twist_t, twist_axis)
                                print(f"扭转角度 cur_twist_angle: {cur_twist_angle}")

                                # 根据旋转轴选择合适的值来设置权重
                                if ch_axis == 'Y':  # 如果选中了 Y 轴，则使用扭转角度
                                    w = _triangular_ratio(cur_twist_angle, W_cur)
                                else:  # 否则使用摆动角度
                                    w = _triangular_ratio(W_cur, W_cur)

                            else:
                                # 如果没有旋转模式或不需要计算摆动和扭转，直接计算角度
                                w = cur_rot_value
                            
                            # 将角度转换为弧度
                            w = math.radians(w)
                            
                            # 防止 NaN
                            if math.isnan(w):
                                w = 0.0

                            # 写入结果
                            psd_set_result_datablock_only(arm, key_rot, w, verbose=False)
                            print(f"记录通道 {ch_axis} 的旋转值（弧度）: {w}")

                    except Exception as e:
                        print(f"Direct Channel Record 出错: {e}")
                        try:
                            key_rot = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                            psd_set_result_datablock_only(arm, key_rot, 0.0, verbose=False)
                        except Exception as e:
                            print(f"写入失败: {e}")

                # 旋转通道计算（如果存在）
                if getattr(entry, 'has_rot', False):
                    try:
                        cur_rot = bone_to_cur_rot.get(bn)
                        if cur_rot is not None:
                            # 如果启用了 cone_enabled -> 使用原先的锥形方法
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

                            # 否则，如果用户选择了旋转通道模式 ->
                            elif getattr(entry, 'rot_channel_mode', 'NONE') and entry.rot_channel_mode != 'NONE':
                                try:
                                    # 把保存的 rest/pose/current 欧拉都转成四元数
                                    q_rest = _euler_deg_to_quat(tuple(entry.rest_rot))
                                    q_pose = _euler_deg_to_quat(tuple(entry.pose_rot))
                                    q_cur = _euler_deg_to_quat(tuple(cur_rot))

                                    # 计算相对于 rest 的相对旋转
                                    q_target = q_rest.inverted() @ q_pose
                                    q_cur_rel = q_rest.inverted() @ q_cur

                                    # 选择扭转轴（局部轴） —— 保留 axis 向量
                                    axis_map = {
                                        'SWING_X_TWIST': Vector((1.0, 0.0, 0.0)),
                                        'SWING_Y_TWIST': Vector((0.0, 1.0, 0.0)),
                                        'SWING_Z_TWIST': Vector((0.0, 0.0, 1.0)),
                                    }
                                    axis = axis_map.get(entry.rot_channel_mode, Vector((0.0, 0.0, 1.0)))

                                    # --- 替换为基于 Euler 分量差的 twist 量（度） ---
                                    axis_idx_map = {'SWING_X_TWIST': 0, 'SWING_Y_TWIST': 1, 'SWING_Z_TWIST': 2}
                                    axis_idx = axis_idx_map.get(entry.rot_channel_mode, 2)

                                    # 注意：entry.rest_rot / entry.pose_rot 是 FloatVectorProperty；cur_rot 来源于采样（Vector）
                                    target_twist_angle = float(entry.pose_rot[axis_idx] - entry.rest_rot[axis_idx])
                                    cur_twist_angle = float(cur_rot[axis_idx] - entry.rest_rot[axis_idx])

                                    # 三角包络映射
                                    w_twist = _triangular_ratio(cur_twist_angle, target_twist_angle)
                                    w = float(max(0.0, min(1.0, w_twist)))


                                except Exception:
                                    # 回退到旧的向量投影法
                                    pose_dir = Vector(entry.pose_rot) - Vector(entry.rest_rot)
                                    cur_rel = Vector(cur_rot) - Vector(entry.rest_rot)
                                    denom = pose_dir.dot(pose_dir)
                                    if denom < 1e-6:
                                        if cur_rel.length < 1e-3:
                                            w = 1.0
                                        else:
                                            w = 0.0
                                    else:
                                        w = cur_rel.dot(pose_dir) / denom
                                        if w < 0.0:
                                            w = 0.0
                                        elif w > 2.0:
                                            w = 0.0
                                        elif w > 1.0:
                                            w = 2.0 - w

                            # 否则使用原始的向量投影法
                            else:
                                pose_dir = Vector(entry.pose_rot) - Vector(entry.rest_rot)
                                cur_rel = cur_rot - Vector(entry.rest_rot)
                                denom = pose_dir.dot(pose_dir)
                                if denom < 1e-6:
                                    if cur_rel.length < 1e-3:
                                        w = 1.0
                                    else:
                                        w = 0.0
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
                # 位移通道计算（如果存在）
                if getattr(entry, 'has_loc', False):
                    try:
                        cur_loc = bone_to_cur_loc.get(bn)
                        if cur_loc is not None:
                            center = Vector(entry.pose_loc)
                            radius = float(getattr(entry, 'loc_radius', 0.0) or 0.0)

                            # 如果 loc_enabled 为 True -> 使用逐轴线性衰减（原始方法）
                            if getattr(entry, 'loc_enabled', False):
                                if radius <= 0.0:
                                    # 需要精确匹配
                                    w_loc = 1.0 if (cur_loc - center).length < 1e-6 else 0.0
                                else:
                                    d = cur_loc - center
                                    wx = max(0.0, 1.0 - abs(d.x) / radius)
                                    wy = max(0.0, 1.0 - abs(d.y) / radius)
                                    wz = max(0.0, 1.0 - abs(d.z) / radius)
                                    w_loc = wx * wy * wz
                            else:
                                # 修改版：逐轴类PSD衰减，忽略pose_loc[i]≈0的轴
                                w_loc = 1.0
                                num_nonzero = 0

                                # 取出rest与pose（都应以 tuple/list 存在 entry）
                                rest_vec = Vector(getattr(entry, 'rest_loc', (0.0, 0.0, 0.0)))
                                pose_vec = Vector(getattr(entry, 'pose_loc', (0.0, 0.0, 0.0)))
                                for i in range(3):
                                    rest_comp = rest_vec[i]
                                    pose_comp = pose_vec[i]
                                    cur_comp = cur_loc[i]

                                    denom = (pose_comp - rest_comp)
                                    # 如果 rest 与 pose 在该轴上几乎相同 -> 无法根据该轴判断，跳过
                                    if abs(denom) < 1e-8:
                                        # 若 cur 与 rest 也几乎相同 -> 视为完全匹配该轴（w_i = 1）
                                        if abs(cur_comp - rest_comp) < 1e-6:
                                            # treat as neutral (multiply by 1)
                                            num_nonzero += 1
                                            continue
                                        else:
                                            # 这一轴无法贡献权重（跳过），但不把整个 w_loc 归零
                                            continue

                                    # 投影到 rest->pose 线段，得到标量比例
                                    w_i = (cur_comp - rest_comp) / denom

                                    # 原来逻辑：负数 -> 0； >2 -> 0； 1..2 -> 对称映射 2 - w_i
                                    if math.isnan(w_i):
                                        w_i = 0.0
                                    if w_i < 0.0:
                                        w_i = 0.0
                                    elif w_i > 2.0:
                                        w_i = 0.0
                                    elif w_i > 1.0:
                                        w_i = 2.0 - w_i

                                    # 保证数值稳定
                                    w_i = max(0.0, min(1.0, w_i))

                                    w_loc *= w_i
                                    num_nonzero += 1

                                # 如果所有轴都被视为“几乎不可判定”（num_nonzero==0），则退回到精确匹配判断
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
                        
                # ---------- 触发器计算 ----------
                # 参数：arm = 当前 armature object
                if hasattr(arm, "psd_triggers"):
                    for trig in arm.psd_triggers:
                        trig.last_weight = 0.0
                        if not trig.enabled:
                            continue
                        # 骨骼存在性检查
                        try:
                            pb_trigger = arm.pose.bones.get(trig.bone_name)
                            pb_target = arm.pose.bones.get(trig.target_bone)
                        except Exception:
                            pb_trigger = None
                            pb_target = None
                        if not pb_trigger or not pb_target:
                            continue

                        # 世界坐标下的骨骼头坐标
                        try:
                            head_trigger_world = arm.matrix_world @ pb_trigger.head
                            head_target_world = arm.matrix_world @ pb_target.head
                        except Exception:
                            # 兼容不同 blender 版本的 API（如果 pb.head 不可用）
                            head_trigger_world = arm.matrix_world @ pb_trigger.bone.head_local
                            head_target_world = arm.matrix_world @ pb_target.bone.head_local

                        d = (head_target_world - head_trigger_world).length
                        r = max(1e-6, float(trig.radius))
                        # 选择衰减类型
                        if trig.falloff == 'SMOOTH':
                            # smoothstep: 3t^2 - 2t^3 on normalized distance t = clamp(d/r,0,1)
                            t = min(max(d / r, 0.0), 1.0)
                            w = 1.0 - (3.0 * t * t - 2.0 * t * t * t)
                            # since when d=0 => t=0 => w=1; when d>=r => t>=1 => w=0
                        else:
                            # 线性：w = clamp(1 - d/r, 0, 1)
                            w = max(0.0, min(1.0, 1.0 - d / r))

                        trig.last_weight = w

                        # 将结果写回 Armature 自定义属性，便于调试或其他模块访问
                        bn = trig.target_bone or ""
                        en = trig.name or ""
                        key_base = f"{PREFIX_RESULT_LOC}{_safe_name(bn)}_{_safe_name(en)}"
                        try:
                            psd_set_result_datablock_only(arm, f"{key_base}_w", float(w), verbose=False)
                        except Exception:
                            pass
                      
                # 记录此条目的性能（毫秒）
                if perf_enabled:
                    t_entry_end = time.perf_counter()
                    dt_ms = max(0.0, (t_entry_end - t_entry_start) * 1000.0) if t_entry_start else 0.0
                    rep_key = None
                    if getattr(entry, 'has_rot', False):
                        rep_key = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                    elif getattr(entry, 'has_loc', False):
                        rep_key = f"{PREFIX_RESULT_LOC}{_safe_name(bn)}_{_safe_name(en)}"
                    else:
                        rep_key = f"{PREFIX_RESULT}{_safe_name(bn)}_{_safe_name(en)}"
                    ent_stats = arm_stats["entries"].setdefault(rep_key, {"hist": [], "last_ms": 0.0, "avg_ms": 0.0})
                    ent_stats["last_ms"] = dt_ms
                    hist = ent_stats["hist"]
                    hist.append(dt_ms)
                    if len(hist) > history_len:
                        hist.pop(0)
                    ent_stats["avg_ms"] = (sum(hist) / len(hist)) if hist else 0.0

            # 记录骨架处理时间
            if perf_enabled:
                t_end_arm = time.perf_counter()
                arm_ms = max(0.0, (t_end_arm - t_start_arm) * 1000.0) if t_start_arm else 0.0
                arm_stats["last_arm_ms"] = arm_ms

        except Exception as e:
            print('PSD计算骨架时出错', getattr(arm, "name", "<unknown>"), e)

    # 记录总时间
    if perf_enabled:
        t_end_total = time.perf_counter()
        total_ms = max(0.0, (t_end_total - t_start_total) * 1000.0) if t_start_total else 0.0
        _psd_perf_stats.setdefault("__global__", {})["last_total_ms"] = total_ms

# --------- 播放检测辅助 ----------
def _is_animation_playing():
    try:
        scr = bpy.context.screen
        return bool(getattr(scr, "is_animation_playing", False))
    except Exception:
        try:
            return bool(bpy.context.scene and getattr(bpy.context.scene, "is_playing", False))
        except Exception:
            return False

# --------- 写入 helper（只写 datablock，不写 object-level；并做 UI 刷新）----------
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

# -------------------------------
# 处理器 / 工具
# -------------------------------
import bpy.app.handlers as handlers

def _handler_name_matches(h, target_name):
    try:
        if getattr(h, "__name__", None) == target_name:
            return True
        if getattr(getattr(h, "__func__", None), "__name__", None) == target_name:
            return True
        return target_name in repr(h)
    except Exception:
        return False

def _remove_handlers_with_name(list_ref, target_name):
    removed = False
    for h in list(list_ref):
        if _handler_name_matches(h, target_name):
            try:
                list_ref.remove(h)
                removed = True
            except Exception:
                pass
    return removed

@handlers.persistent
def psd_depsgraph_handler(depsgraph):
    try:
        sc = bpy.context.scene if bpy.context and bpy.context.scene else None
        if sc and getattr(sc, 'psd_mode', 'AUTO') == 'FORCE_TIMER':
            return
        if not _is_animation_playing() and (not sc or getattr(sc, 'psd_mode', 'AUTO') != 'FORCE_PLAY'):
            return
        _psd_compute_all(depsgraph=depsgraph)
    except Exception as e:
        print("PSD depsgraph处理器错误:", e)

@handlers.persistent
def psd_frame_handler(scene):
    try:
        if scene and getattr(scene, 'psd_mode', 'AUTO') == 'FORCE_TIMER':
            return
        if _is_animation_playing() or (scene and getattr(scene, 'psd_mode', 'FORCE_PLAY')):
            deps = bpy.context.evaluated_depsgraph_get()
            _psd_compute_all(depsgraph=deps)
        else:
            _psd_compute_all(depsgraph=None)
    except Exception as e:
        print("PSD帧变化处理器错误:", e)

# -------------------------------
# 计时器 + 消息总线集成
# -------------------------------
_psd_timer_registered = False
_msgbus_owner = object()
_msgbus_subscribed = False

def _get_scene_for_timer():
    if bpy.context and getattr(bpy.context, "scene", None):
        return bpy.context.scene
    if bpy.data.scenes:
        return bpy.data.scenes[0]
    return None

def _psd_timer_func():
    global _psd_timer_registered
    _psd_timer_registered = True
    try:
        sc = _get_scene_for_timer()
        if not sc or not getattr(sc, 'psd_running', False):
            _psd_timer_registered = False
            return None
        if getattr(sc, 'psd_mode', 'AUTO') == 'FORCE_PLAY':
            _psd_timer_registered = False
            return None
        if getattr(sc, 'psd_mode', 'AUTO') == 'AUTO' and _is_animation_playing():
            _psd_timer_registered = False
            return None
        try:
            _psd_compute_all(depsgraph=None)
        except Exception as e:
            print("PSD计时器计算错误:", e)
        hz = int(getattr(sc, 'psd_idle_hz', 10) or 10)
        if hz < 1:
            hz = 1
        return 1.0 / float(hz)
    except Exception as e:
        _psd_timer_registered = False
        print("PSD计时器异常:", e)
        return None

def _on_play_changed():
    global _psd_timer_registered
    try:
        sc = bpy.context.scene if bpy.context and bpy.context.scene else None
        if not sc or not getattr(sc, 'psd_running', False):
            return
        mode = getattr(sc, 'psd_mode', 'AUTO')
        if mode == 'FORCE_PLAY':
            _psd_timer_registered = False
            return
        if mode == 'FORCE_TIMER':
            if not _psd_timer_registered:
                try:
                    bpy.app.timers.register(_psd_timer_func, first_interval=0.0)
                    _psd_timer_registered = True
                except Exception:
                    pass
            return
        if _is_animation_playing():
            _psd_timer_registered = False
        else:
            if not _psd_timer_registered:
                try:
                    bpy.app.timers.register(_psd_timer_func, first_interval=0.0)
                    _psd_timer_registered = True
                except Exception:
                    pass
    except Exception as e:
        print("PSD消息总线播放状态变化处理器异常:", e)

def _subscribe_msgbus_for_play_change():
    global _msgbus_subscribed
    if _msgbus_subscribed:
        return
    try:
        bpy.msgbus.subscribe_rna(
            key=(bpy.types.Screen, "is_animation_playing"),
            owner=_msgbus_owner,
            notify=_on_play_changed,
        )
        _msgbus_subscribed = True
    except Exception as e:
        print("PSD消息总线订阅失败:", e)
        _msgbus_subscribed = False

def _unsubscribe_msgbus():
    global _msgbus_subscribed
    try:
        bpy.msgbus.clear_by_owner(_msgbus_owner)
    except Exception:
        pass
    _msgbus_subscribed = False

# -------------------------------
# 启动 / 停止 操作符
# -------------------------------
class PSDStartOperator(bpy.types.Operator):
    bl_idname = "object.psd_start"
    bl_label = "启动PSD校正器"

    def execute(self, context):
        global _msgbus_subscribed, _psd_timer_registered
        sc = context.scene
        if not sc.psd_running:
            try:
                _remove_handlers_with_name(handlers.depsgraph_update_post, psd_depsgraph_handler.__name__)
            except Exception:
                pass
            try:
                _remove_handlers_with_name(handlers.frame_change_post, psd_frame_handler.__name__)
            except Exception:
                pass

            try:
                if not any(_handler_name_matches(h, psd_depsgraph_handler.__name__) for h in handlers.depsgraph_update_post):
                    handlers.depsgraph_update_post.append(psd_depsgraph_handler)
            except Exception:
                pass
            try:
                if not any(_handler_name_matches(h, psd_frame_handler.__name__) for h in handlers.frame_change_post):
                    handlers.frame_change_post.append(psd_frame_handler)
            except Exception:
                pass

            sc.psd_running = True

            mode = getattr(sc, 'psd_mode', 'AUTO')

            if mode == 'AUTO':
                _subscribe_msgbus_for_play_change()
                try:
                    _on_play_changed()
                except Exception:
                    pass
            elif mode == 'FORCE_TIMER':
                if not _psd_timer_registered:
                    try:
                        bpy.app.timers.register(_psd_timer_func, first_interval=0.0)
                        _psd_timer_registered = True
                    except Exception:
                        pass
            elif mode == 'FORCE_PLAY':
                _psd_timer_registered = False

            self.report({'INFO'}, "PSD校正器已启动 (处理器已激活)")
        else:
            self.report({'INFO'}, "PSD校正器已在运行")
        return {'FINISHED'}

class PSDStopOperator(bpy.types.Operator):
    bl_idname = "object.psd_stop"
    bl_label = "停止PSD校正器"

    def execute(self, context):
        global _psd_timer_registered, _msgbus_subscribed
        try:
            if psd_depsgraph_handler in handlers.depsgraph_update_post:
                handlers.depsgraph_update_post.remove(psd_depsgraph_handler)
        except Exception:
            pass
        try:
            _remove_handlers_with_name(handlers.depsgraph_update_post, psd_depsgraph_handler.__name__)
        except Exception:
            pass
        try:
            if psd_frame_handler in handlers.frame_change_post:
                handlers.frame_change_post.remove(psd_frame_handler)
        except Exception:
            pass
        try:
            _remove_handlers_with_name(handlers.frame_change_post, psd_frame_handler.__name__)
        except Exception:
            pass

        try:
            context.scene.psd_running = False
        except Exception:
            pass

        _psd_timer_registered = False
        _unsubscribe_msgbus()

        self.report({'INFO'}, "PSD校正器已停止 (处理器已移除)")
        return {'FINISHED'}

# -------------------------------
# 触发器 (位移)
# -------------------------------

class PSDBoneTrigger(bpy.types.PropertyGroup):
    """单个触发器条目：放在触发骨骼头周围一个半径（球形），当目标骨骼头进入范围时产生权重"""
    name: StringProperty(name="Name", default="Trigger")
    bone_name: StringProperty(name="Trigger Bone", default="")     # 触发器骨骼（创建时默认活动骨骼）
    target_bone: StringProperty(name="Target Bone", default="")    # 被检测的目标骨骼
    enabled: BoolProperty(name="Enabled", default=True)
    radius: FloatProperty(name="Radius", default=0.2, min=0.0, description="Trigger radius (world units)")
    # 可选：是否线性/平滑衰减（enum），当前仅用线性
    falloff: EnumProperty(
        name="Falloff",
        items=[
            ('LINEAR', "Linear", "Linear falloff (1 - d / r)"),
            ('SMOOTH', "Smoothstep", "Smoothstep falloff"),
        ],
        default='LINEAR'
    )
    # 运行时结果（只读供 UI 显示），不会被序列化为复杂对象，但会保存为小数
    last_weight: FloatProperty(name="Last Weight", default=0.0)


class PSD_OT_AddTrigger(bpy.types.Operator):
    bl_idname = "psd.add_trigger"
    bl_label = "Add Trigger"
    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请选择骨架对象")
            return {'CANCELLED'}
        pb = None
        if context.mode.startswith('POSE'):
            pb = context.active_pose_bone
        # 新条目
        t = arm.psd_triggers.add()
        t.name = "Trigger"
        if pb:
            t.bone_name = pb.name
        else:
            t.bone_name = ""
        arm.psd_trigger_index = len(arm.psd_triggers) - 1
        return {'FINISHED'}

class PSD_OT_RemoveTrigger(bpy.types.Operator):
    bl_idname = "psd.remove_trigger"
    bl_label = "Remove Trigger"
    def execute(self, context):
        arm = context.object
        idx = arm.psd_trigger_index
        if not arm or idx < 0 or idx >= len(arm.psd_triggers):
            return {'CANCELLED'}
        #UI更新
        try:
            key = arm.psd_triggers[idx]
            key_base = f"{PREFIX_RESULT_LOC}{_safe_name(key.target_bone)}_{_safe_name(key.name)}_w"
            arm_db = bpy.data.armatures.get(arm.data.name)
            if arm_db:
                if key_base in arm_db:
                    print("success del " + key_base)
                    del arm_db[key_base]
            arm.psd_triggers.remove(idx)
            arm.psd_trigger_index = min(max(0, idx-1), len(arm.psd_triggers)-1)
        except Exception as e:
            self.report({'ERROR'}, f"移除条目时出错: {e}")
            return {'CANCELLED'}
        return {'FINISHED'}


class PSD_OT_SelectTriggerBone(bpy.types.Operator):
    bl_idname = "psd.select_trigger_bone"
    bl_label = "Select Trigger/Target Bone"
    mode: EnumProperty(items=[('TRIGGER','Trigger','Set trigger bone'), ('TARGET','Target','Set target bone')], default='TRIGGER')
    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            return {'CANCELLED'}
        pb = context.active_pose_bone if context.mode.startswith('POSE') else None
        idx = arm.psd_trigger_index
        if idx < 0 or idx >= len(arm.psd_triggers):
            self.report({'ERROR'}, "请先选中一个触发器条目")
            return {'CANCELLED'}
        if not pb:
            self.report({'ERROR'}, "请在 Pose 模式选中一个骨骼头")
            return {'CANCELLED'}
        if self.mode == 'TRIGGER':
            arm.psd_triggers[idx].bone_name = pb.name
        else:
            arm.psd_triggers[idx].target_bone = pb.name
        return {'FINISHED'}


# -------------------------------
# 捕捉操作符 (旋转 / 位移)
# -------------------------------
class PSDCaptureRest(bpy.types.Operator):
    bl_idname = "psd.capture_rest"
    bl_label = "捕捉静止旋转"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "骨骼过滤器中未选择骨骼。请先在骨骼过滤器列表中添加/选择一个骨骼。")
            return {'CANCELLED'}
        try:
            degs = _capture_bone_local_rotation_deg(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_rest_bone = selected_bone
            context.scene.psd_temp_rest = degs
            self.report({'INFO'}, f"已为 {selected_bone} 捕捉静止旋转: {degs}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"捕捉静止旋转失败: {ex}")
            return {'CANCELLED'}

class PSDCaptureRotation(bpy.types.Operator):
    bl_idname = "psd.capture_rotation"
    bl_label = "捕捉姿态旋转"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "骨骼过滤器中未选择骨骼。请先在骨骼过滤器列表中添加/选择一个骨骼。")
            return {'CANCELLED'}
        try:
            degs = _capture_bone_local_rotation_deg(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_pose_bone = selected_bone
            context.scene.psd_temp_pose = degs
            self.report({'INFO'}, f"已为 {selected_bone} 捕捉姿态旋转: {degs}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"捕捉姿态旋转失败: {ex}")
            return {'CANCELLED'}

class PSDCaptureLocationRest(bpy.types.Operator):
    bl_idname = "psd.capture_loc_rest"
    bl_label = "捕捉静止位置"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "骨骼过滤器中未选择骨骼。请先在骨骼过滤器列表中添加/选择一个骨骼。")
            return {'CANCELLED'}
        try:
            rest_loc, _ = _capture_bone_local_translation(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_loc_rest_bone = selected_bone
            context.scene.psd_temp_loc_rest = tuple(rest_loc)
            self.report({'INFO'}, f"已为 {selected_bone} 捕捉静止位置: {tuple(rest_loc)}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"捕捉静止位置失败: {ex}")
            return {'CANCELLED'}

class PSDCaptureLocation(bpy.types.Operator):
    bl_idname = "psd.capture_location"
    bl_label = "捕捉姿态位置"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "骨骼过滤器中未选择骨骼。请先在骨骼过滤器列表中添加/选择一个骨骼。")
            return {'CANCELLED'}
        try:
            _, pose_loc = _capture_bone_local_translation(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_loc_bone = selected_bone
            context.scene.psd_temp_loc = tuple(pose_loc)
            self.report({'INFO'}, f"已为 {selected_bone} 捕捉姿态位置: {tuple(pose_loc)}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"捕捉姿态位置失败: {ex}")
            return {'CANCELLED'}

# -------------------------------
# 将捕捉的静止+姿态保存为SavedPose条目 (分离为旋转/位移)
# -------------------------------
class PSDSaveCapturedRotationEntry(bpy.types.Operator):
    bl_idname = "psd.save_captured_rotation"
    bl_label = "保存捕捉的旋转条目"

    entry_name: bpy.props.StringProperty(name="条目名称", default="default")

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        arm = context.object
        scene = context.scene
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}

        bone_r = getattr(scene, 'psd_temp_rest_bone', '')
        bone_p = getattr(scene, 'psd_temp_pose_bone', '')

        has_rot = bool(bone_p and bone_p != '<NONE>')
        if not has_rot:
            self.report({'ERROR'}, "请先捕捉一个旋转 (必需)")
            return {'CANCELLED'}

        if bone_r and bone_r != bone_p and bone_r != '':
            self.report({'WARNING'}, "旋转的静止/姿态来自不同骨骼；如有需要，此条目的静止值将设为(0,0,0)")
            rest_vals_rot = (0.0,0.0,0.0)
        else:
            rest_vals_rot = tuple(getattr(scene, 'psd_temp_rest', (0.0,0.0,0.0)))

        pose_vals_rot = tuple(getattr(scene, 'psd_temp_pose', (0.0,0.0,0.0)))
        bone_name_rot = getattr(scene, 'psd_temp_pose_bone', '')

        # 检查该骨骼是否已存在同名条目
        for existing in arm.psd_saved_poses:
            if existing.bone_name == bone_name_rot and existing.name == self.entry_name:
                self.report({'ERROR'}, f"骨骼 {bone_name_rot} 的条目 '{self.entry_name}' 已存在")
                return {'CANCELLED'}

        # 创建条目 (仅旋转)
        new = arm.psd_saved_poses.add()
        new.name = self.entry_name if self.entry_name else "default"
        new.bone_name = bone_name_rot
        new.rest_rot = rest_vals_rot
        new.pose_rot = pose_vals_rot
        new.is_direct_channel = False
        new.has_rot = True
        # 确保位移字段为空/禁用
        new.rest_loc = (0.0,0.0,0.0)
        new.pose_loc = (0.0,0.0,0.0)
        new.has_loc = False
        new.loc_enabled = False
        new.loc_radius = 0.1

        arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1

        # 仅清除旋转的临时捕捉数据
        scene.psd_temp_rest = (0.0,0.0,0.0)
        scene.psd_temp_pose = (0.0,0.0,0.0)
        scene.psd_temp_rest_bone = ''
        scene.psd_temp_pose_bone = ''

        # 创建并初始化旋转结果的键
        entry_key_rot = f"{PREFIX_RESULT}{_safe_name(new.bone_name)}_{_safe_name(new.name)}"
        try:
            arm.data[entry_key_rot] = 0.0
        except Exception:
            pass

        self.report({'INFO'}, f"已在骨架 {arm.name} 上为骨骼 {bone_name_rot} 保存旋转条目 '{new.name}'")
        return {'FINISHED'}

class PSDSaveCapturedLocationEntry(bpy.types.Operator):
    bl_idname = "psd.save_captured_location"
    bl_label = "保存捕捉的位置条目"

    entry_name: bpy.props.StringProperty(name="条目名称", default="default")

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        arm = context.object
        scene = context.scene
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}

        bone_loc_r = getattr(scene, 'psd_temp_loc_rest_bone', '')
        bone_loc_p = getattr(scene, 'psd_temp_loc_bone', '')

        has_loc  = bool(bone_loc_p and bone_loc_p != '<NONE>')
        if not has_loc:
            self.report({'ERROR'}, "请先捕捉一个位置 (必需)")
            return {'CANCELLED'}

        if bone_loc_r and bone_loc_r != bone_loc_p and bone_loc_r != '':
            self.report({'WARNING'}, "位置的静止/姿态来自不同骨骼；如有需要，此条目的静止值将设为(0,0,0)")
            rest_vals_loc = (0.0,0.0,0.0)
        else:
            rest_vals_loc = tuple(getattr(scene, 'psd_temp_loc_rest', (0.0,0.0,0.0)))

        pose_vals_loc = tuple(getattr(scene, 'psd_temp_loc', (0.0,0.0,0.0)))
        bone_name_loc = getattr(scene, 'psd_temp_loc_bone', '')

        # 检查该骨骼是否已存在同名条目
        for existing in arm.psd_saved_poses:
            if existing.bone_name == bone_name_loc and existing.name == self.entry_name:
                self.report({'ERROR'}, f"骨骼 {bone_name_loc} 的条目 '{self.entry_name}' 已存在")
                return {'CANCELLED'}

        # 创建条目 (仅位移)
        new = arm.psd_saved_poses.add()
        new.name = self.entry_name if self.entry_name else "default"
        new.bone_name = bone_name_loc
        new.is_direct_channel = False
        # 旋转字段为空/禁用
        new.rest_rot = (0.0,0.0,0.0)
        new.pose_rot = (0.0,0.0,0.0)
        new.has_rot = False
        # 设置位移字段
        new.rest_loc = rest_vals_loc
        new.pose_loc = pose_vals_loc
        new.has_loc = True
        # 保留默认衰减设置 (用户后续可编辑)
        new.loc_enabled = getattr(new, 'loc_enabled', False)
        new.loc_radius = getattr(new, 'loc_radius', 0.1)

        arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1

        # 仅清除位移的临时捕捉数据
        scene.psd_temp_loc_rest = (0.0,0.0,0.0)
        scene.psd_temp_loc = (0.0,0.0,0.0)
        scene.psd_temp_loc_rest_bone = ''
        scene.psd_temp_loc_bone = ''

        # 创建并初始化位移结果的键
        entry_key_loc = f"{PREFIX_RESULT_LOC}{_safe_name(new.bone_name)}_{_safe_name(new.name)}"
        try:
            arm.data[entry_key_loc] = 0.0
        except Exception:
            pass

        self.report({'INFO'}, f"已在骨架 {arm.name} 上为骨骼 {bone_name_loc} 保存位置条目 '{new.name}'")
        return {'FINISHED'}

class PSDRemoveSavedEntry(bpy.types.Operator):
    bl_idname = "psd.remove_saved_entry"
    bl_label = "移除已保存的条目"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        
        idx = arm.psd_saved_pose_index
        if idx < 0 or idx >= len(arm.psd_saved_poses):
            self.report({'ERROR'}, "未选择已保存的姿态")
            return {'CANCELLED'}
        
        try:
            # 获取要删除的条目
            entry = arm.psd_saved_poses[idx]
            entry_key_rot = f"{PREFIX_RESULT}{_safe_name(entry.bone_name)}_{_safe_name(entry.name)}"
            entry_key_loc = f"{PREFIX_RESULT_LOC}{_safe_name(entry.bone_name)}_{_safe_name(entry.name)}"
            
            # 改进1: 显式获取 Armature Datablock，与写入函数保持一致
            arm_db = bpy.data.armatures.get(arm.data.name)

            if arm_db:
                # 从数据块中安全地删除自定义属性
                if entry_key_rot in arm_db:
                    del arm_db[entry_key_rot]
                if entry_key_loc in arm_db:
                    del arm_db[entry_key_loc]
            
            # 从UI列表对应的集合中移除该条目
            arm.psd_saved_poses.remove(idx)
            
            # 更新UI列表的选中索引
            new_len = len(arm.psd_saved_poses)
            arm.psd_saved_pose_index = min(max(0, idx - 1), new_len - 1) if new_len > 0 else -1
            
            self.report({'INFO'}, f"已移除条目 '{entry.name}'")

        except Exception as e:
            # 改进2: 捕获所有潜在错误并报告，防止静默失败
            self.report({'ERROR'}, f"移除条目时出错: {e}")
            return {'CANCELLED'}

        return {'FINISHED'}

# -------------------------------
# 已保存姿态的自定义UI列表
# -------------------------------
class PSDSavedPoseUIList(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            txt = item.name
            if getattr(item, 'is_direct_channel', False):
                txt += f" [Direct {item.channel_axis}]"
            tags = []
            if getattr(item, 'has_rot', False):
                tags.append("R")
            if getattr(item, 'has_loc', False):
                tags.append("L")
            if tags:
                txt += f" [{' '.join(tags)}]"
            layout.label(text=txt)
        elif self.layout_type == 'GRID':
            layout.alignment = 'CENTER'
            layout.label(text="", icon='BONE_DATA')

    def filter_items(self, context, data, propname):
        items = getattr(data, propname)
        ln = len(items)

        # 默认：全部隐藏（位标志数组长度必须为 ln）
        filtered = [0] * ln
        new_order = list(range(ln))  # 默认新顺序为原始顺序

        # 获取骨骼过滤器里选中的骨骼
        selected_bone = '<NONE>'
        arm = context.object
        if arm and arm.type == 'ARMATURE':
            try:
                idx = int(getattr(arm, 'psd_bone_pairs_index', 0))
                if 0 <= idx < len(arm.psd_bone_pairs):
                    selected_bone = arm.psd_bone_pairs[idx].bone_name or '<NONE>'
            except Exception:
                selected_bone = '<NONE>'

        if selected_bone == '<NONE>':
            # 显示所有，保留原始顺序
            filtered = [self.bitflag_filter_item] * ln
            new_order = list(range(ln))
        else:
            # 只显示匹配 bone_name 的条目，把匹配项排在前面，其余项按原顺序跟在后面
            matching = []
            for i, item in enumerate(items):
                if item.bone_name == selected_bone:
                    filtered[i] = self.bitflag_filter_item
                    matching.append(i)
                else:
                    filtered[i] = 0
            # new_order 必须是长度为 ln 的排列：先 matching，再剩下的
            remaining = [i for i in range(ln) if i not in matching]
            new_order = matching + remaining

        return filtered, new_order


# -------------------------------
# UI 面板 (带位移控件和性能调试)
# -------------------------------
class PSDPanel(bpy.types.Panel):
    bl_label = "PSD校正器"
    bl_idname = "VIEW3D_PT_psd_corrector_armature"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'PSD'

    @classmethod
    def poll(cls, context):
        obj = context.object
        return (obj is not None) and (obj.type == 'ARMATURE')

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        arm = context.object

        layout.label(text=f"骨架: {arm.name}")
        layout.separator()

        # 骨骼对过滤器 UI (逐骨架)
        layout.label(text='骨骼过滤器 (若非空，则仅计算这些骨骼):')
        row = layout.row()
        row.template_list("PSDBonePairUIList", "psd_bone_pairs", arm, "psd_bone_pairs", arm, "psd_bone_pairs_index", rows=6)
        col = row.column(align=True)
        col.operator('psd.add_bone_pair', icon='ADD', text='')
        col.operator('psd.remove_bone_pair', icon='REMOVE', text='')
        col.separator()
        col.operator('psd.move_bone_pair', icon='TRIA_UP', text='').direction = 'UP'
        col.operator('psd.move_bone_pair', icon='TRIA_DOWN', text='').direction = 'DOWN'

        layout.separator()
        
        # 导出 / 导入（JSON）
        layout.separator()
        row2 = layout.row(align=True)
        row2.operator('psd.export_config', icon='EXPORT', text='导出配置')
        row2.operator('psd.import_config', icon='IMPORT', text='导入配置')
        
        layout.separator()

        # 显示骨骼过滤器中当前选择的骨骼
        selected_bone = _get_selected_pair_bone(context)
        layout.label(text=f"已选骨骼: {selected_bone}")
        # 捕捉按钮: 静止/旋转/位移
        row = layout.row(align=True)
        row.operator('psd.capture_rest', icon='REW', text='捕捉静止旋转')
        row.operator('psd.capture_rotation', icon='POSE_HLT', text='捕捉姿态旋转')
        row = layout.row(align=True)
        row.operator('psd.capture_loc_rest', icon='EMPTY_AXIS', text='捕捉静止位置')
        row.operator('psd.capture_location', icon='EMPTY_DATA', text='捕捉姿态位置')
        
        layout.separator()
        layout.label(text="骨骼触发器", icon='EMPTY_DATA')
        row = layout.row()
        row.template_list(
            "PSDBoneTriggerUIList", "psd_triggers",
            arm, "psd_triggers",
            arm, "psd_trigger_index",
            rows=4
        )
        col = row.column(align=True)
        col.operator("psd.add_trigger", icon='ADD', text="")
        col.operator("psd.remove_trigger", icon='REMOVE', text="")
        col.separator()
        col.operator("psd.select_trigger_bone", icon='EYEDROPPER', text="").mode = 'TRIGGER'
        col.operator("psd.select_trigger_bone", icon='RESTRICT_SELECT_OFF', text="").mode = 'TARGET'

        # Below the list, show properties of selected trigger
        idx = arm.psd_trigger_index if arm.psd_trigger_index >= 0 and arm.psd_trigger_index < len(arm.psd_triggers) else -1
        if idx != -1:
            trig = arm.psd_triggers[idx]
            layout.prop(trig, "enabled")
            layout.prop(trig, "name")
            # show bone names (readonly) and allow radius edit
            layout.label(text=f"Trigger Bone: {trig.bone_name}")
            layout.prop(trig, "target_bone", text="Target Bone")
            layout.prop(trig, "radius")
            layout.prop(trig, "falloff")
            roww = layout.row()
            roww.label(text=f"实时权重: {trig.last_weight:.3f}")


        layout.separator()
        # 显示临时捕捉的数据
        if scene.psd_temp_pose_bone:
            p = scene.psd_temp_pose
            layout.label(text=f"已捕捉旋转: {scene.psd_temp_pose_bone} (X={p[0]:.2f}°, Y={p[1]:.2f}°, Z={p[2]:.2f}°)")
        else:
            layout.label(text='无已捕捉的旋转')
        if scene.psd_temp_rest_bone:
            r = scene.psd_temp_rest
            layout.label(text=f"已捕捉静止旋转: {scene.psd_temp_rest_bone} (X={r[0]:.2f}°, Y={r[1]:.2f}°, Z={r[2]:.2f}°)")
        if scene.psd_temp_loc_bone:
            l = scene.psd_temp_loc
            layout.label(text=f"已捕捉位置: {scene.psd_temp_loc_bone} (X={l[0]:.4f}, Y={l[1]:.4f}, Z={l[2]:.4f})")
        else:
            layout.label(text='无已捕捉的位置')
        if scene.psd_temp_loc_rest_bone:
            lr = scene.psd_temp_loc_rest
            layout.label(text=f"已捕捉静止位置: {scene.psd_temp_loc_rest_bone} (X={lr[0]:.4f}, Y={lr[1]:.4f}, Z={lr[2]:.4f})")

        # 分离的保存按钮
        layout.separator()
        row = layout.row(align=True)
        row.operator('psd.save_captured_rotation', icon='FILE_TICK', text='保存旋转条目')
        row.operator('psd.save_captured_location', icon='FILE_TICK', text='保存位置条目')
        row = layout.row(align=True)
        row.operator('psd.record_channel_x', text="Record X Channel")
        row.operator('psd.record_channel_y', text="Record Y Channel")
        row.operator('psd.record_channel_z', text="Record Z Channel")
        layout.separator()
        bone_name = selected_bone if selected_bone != '<NONE>' else '无'
        layout.label(text=f'骨骼 {bone_name} 的已保存姿态条目:')
        row = layout.row()
        row.template_list('PSDSavedPoseUIList', 'psd_saved_poses', arm, 'psd_saved_poses', arm, 'psd_saved_pose_index')
        col = row.column(align=True)
        col.operator('psd.remove_saved_entry', icon='REMOVE', text='')

        # 显示选中条目的详细信息
        if arm.psd_saved_poses and arm.psd_saved_pose_index >= 0 and arm.psd_saved_pose_index < len(arm.psd_saved_poses):
            e = arm.psd_saved_poses[arm.psd_saved_pose_index]
            if e.bone_name == selected_bone or selected_bone == '<NONE>':
                layout.prop(e, 'name', text="条目名称")
                layout.label(text=f"骨骼: {e.bone_name}")

                # 旋转详情
                if getattr(e, 'is_direct_channel', False):
                    layout.label(text=f"Direct Record Channel: {e.channel_axis}")
                    layout.prop(e, 'record_rot_channel_mode', text='record旋转通道模式')
                else:
                    layout.label(text=f"静止旋转: X={e.rest_rot[0]:.2f}°, Y={e.rest_rot[1]:.2f}°, Z={e.rest_rot[2]:.2f}°")
                    layout.label(text=f"姿态旋转: X={e.pose_rot[0]:.2f}°, Y={e.pose_rot[1]:.2f}°, Z={e.pose_rot[2]:.2f}°")
                    layout.prop(e, 'has_rot', text='包含旋转')
                    layout.prop(e, 'cone_enabled', text='启用锥形衰减')
                    if e.cone_enabled:
                        layout.prop(e, 'cone_angle', text='锥角 (度)')
                        layout.prop(e, 'cone_axis', text='锥体轴向')

                    # 新增：旋转通道模式 UI
                    layout.prop(e, 'rot_channel_mode', text='旋转通道模式')

                    layout.separator()
                    # 位移详情
                    layout.label(text=f"静止位置: X={e.rest_loc[0]:.4f}, Y={e.rest_loc[1]:.4f}, Z={e.rest_loc[2]:.4f}")
                    layout.label(text=f"姿态位置: X={e.pose_loc[0]:.4f}, Y={e.pose_loc[1]:.4f}, Z={e.pose_loc[2]:.4f}")
                    layout.prop(e, 'has_loc', text='包含位移')
                    layout.prop(e, 'loc_enabled', text='启用位置轴向衰减')
                    if e.loc_enabled:
                        layout.prop(e, 'loc_radius', text='轴向半径')

        layout.separator()

        # 显示存储在armature.data中的所有PSD结果 (旋转/位移):
        # PSD 结果显示（可收合、带搜索/排序/上限）
        layout.separator()
        layout.label(text='骨架数据中的PSD结果 (旋转/位移):')
        # 显示开关（用户决定是否展开显示）
        layout.prop(scene, "psd_show_results", text="展开显示所有 PSD 结果")

        if scene.psd_show_results:
            box = layout.box()
            # 搜索栏和控制项
            row = box.row(align=True)
            row.prop(scene, "psd_results_search", text="", icon='VIEWZOOM')
            row = box.row(align=True)
            row.prop(scene, "psd_results_sort_reverse", text="倒序")
            row.prop(scene, "psd_results_limit", text="上限")

            # 收集所有结果到列表（安全地把 keys() 转为 list）
            results = []
            try:
                a_stats = _psd_perf_stats.get(arm.name, {}) if isinstance(_psd_perf_stats, dict) else {}
                entries_stats = a_stats.get("entries", {}) if isinstance(a_stats, dict) else {}
                for k in list(arm.data.keys()):
                    if isinstance(k, str) and (k.startswith(PREFIX_RESULT) or k.startswith(PREFIX_RESULT_LOC)):
                        # 计算短名与 display 标识
                        if k.startswith(PREFIX_RESULT):
                            short = k[len(PREFIX_RESULT):]
                        elif k.startswith(PREFIX_RESULT_LOC):
                            short = k[len(PREFIX_RESULT_LOC):] + " (位移)"
                        else:
                            short = k
                        try:
                            v = float(arm.data.get(k, 0.0))
                        except Exception:
                            v = 0.0
                        ent = entries_stats.get(k, {}) if entries_stats else {}
                        last_ms = float(ent.get("last_ms", 0.0) or 0.0)
                        avg_ms = float(ent.get("avg_ms", 0.0) or 0.0)
                        results.append({"key": k, "short": short, "value": v, "last_ms": last_ms, "avg_ms": avg_ms})
            except Exception:
                results = []

            # 过滤（search 支持 key/short 的不区分大小写子串匹配）
            s = (scene.psd_results_search or "").strip().lower()
            if s:
                results = [r for r in results if (s in (r["key"] or "").lower()) or (s in (r["short"] or "").lower())]

            # 排序
            # 排序（仅按短名）
            rev = bool(getattr(scene, "psd_results_sort_reverse", False))
            results.sort(key=lambda r: (r["short"] or "").lower(), reverse=rev)


            # 显示（受上限限制）
            limit = int(getattr(scene, "psd_results_limit", 200) or 200)
            if limit < 1:
                limit = 1
            if limit > 5000:
                limit = 5000

            if not results:
                box.label(text="<无匹配项>")
            else:
                shown = 0
                for r in results:
                    if shown >= limit:
                        break
                    perf_note = ""
                    if scene.psd_perf_enabled:
                        perf_note = f"  ({r['avg_ms']:.2f}ms avg / {r['last_ms']:.2f}ms last)"
                    box.label(text=f"{r['short']} = {r['value']:.4f}{perf_note}")
                    shown += 1

            # 如果有被过滤掉但存在更多匹配，显示提示
            total_matches = len(results)
            if total_matches > limit:
                box.label(text=f"... 显示 {limit}/{total_matches} 条结果，缩小搜索或增加上限以查看更多")
        else:
            layout.label(text="(已折叠 — 打开 '展开显示所有 PSD 结果' 查看)")



        layout.separator()
        layout.label(text="模式:")
        layout.prop(scene, "psd_mode", text="")

        if scene.psd_mode in ('AUTO', 'FORCE_TIMER'):
            layout.prop(scene, "psd_idle_hz", text="空闲频率(Hz)", slider=True)

        layout.separator()
        layout.label(text="调试/性能:")
        layout.prop(scene, "psd_perf_enabled", text="显示性能调试信息")
        if scene.psd_perf_enabled:
            layout.prop(scene, "psd_perf_history_len", text="历史记录长度")
            arm_stats = _psd_perf_stats.get(arm.name)
            if arm_stats:
                layout.label(text=f"骨架性能: {arm_stats.get('last_arm_ms',0.0):.2f} ms")
                global_total = _psd_perf_stats.get("__global__", {}).get("last_total_ms", 0.0)
                layout.label(text=f"总计算耗时: {global_total:.2f} ms")
                entries = arm_stats.get("entries", {})
                if entries:
                    sorted_items = sorted(entries.items(), key=lambda kv: kv[1].get("avg_ms", 0.0), reverse=True)
                    layout.label(text="平均延迟最高的结果:")
                    for k, s in sorted_items[:10]:
                        shortk = k
                        if k.startswith(PREFIX_RESULT):
                            shortk = k[len(PREFIX_RESULT):]
                        elif k.startswith(PREFIX_RESULT_LOC):
                            shortk = k[len(PREFIX_RESULT_LOC):] + " (位移)"
                        layout.label(text=f"{shortk}: {s.get('avg_ms',0.0):.2f}ms 平均 / {s.get('last_ms',0.0):.2f}ms 上次")
            else:
                layout.label(text="暂无性能数据。请开启调试模式并运行PSD。")

        row2 = layout.row(align=True)
        if scene.psd_running:
            row2.operator('object.psd_stop', icon='CANCEL')
        else:
            row2.operator('object.psd_start', icon='PLAY')

        layout.label(text='(静止/姿态捕捉数据保存在骨架上；骨架数据接收PSD浮点值)')

# -------------------------------
# 注册
# -------------------------------

class PSDRecordChannelX(bpy.types.Operator):
    bl_idname = "psd.record_channel_x"
    bl_label = "Record X Channel"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "骨骼过滤器中未选择骨骼。请先在骨骼过滤器列表中添加/选择一个骨骼。")
            return {'CANCELLED'}
        name = "record_X"
        # Check if exists
        for existing in arm.psd_saved_poses:
            if existing.bone_name == selected_bone and existing.name == name:
                self.report({'ERROR'}, f"骨骼 {selected_bone} 的条目 '{name}' 已存在")
                return {'CANCELLED'}
        try:
            new = arm.psd_saved_poses.add()
            new.name = name
            new.bone_name = selected_bone
            new.is_direct_channel = True
            new.channel_axis = 'X'
            new.has_rot = False
            new.has_loc = False
            arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1
            # Initialize result key
            key = f"{PREFIX_RESULT}{_safe_name(selected_bone)}_{_safe_name(name)}"
            try:
                arm.data[key] = 0.0
            except Exception:
                pass
            self.report({'INFO'}, f"Added {name} for {selected_bone}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"记录失败: {ex}")
            return {'CANCELLED'}

class PSDRecordChannelY(bpy.types.Operator):
    bl_idname = "psd.record_channel_y"
    bl_label = "Record Y Channel"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "骨骼过滤器中未选择骨骼。请先在骨骼过滤器列表中添加/选择一个骨骼。")
            return {'CANCELLED'}
        name = "record_Y"
        # Check if exists
        for existing in arm.psd_saved_poses:
            if existing.bone_name == selected_bone and existing.name == name:
                self.report({'ERROR'}, f"骨骼 {selected_bone} 的条目 '{name}' 已存在")
                return {'CANCELLED'}
        try:
            new = arm.psd_saved_poses.add()
            new.name = name
            new.bone_name = selected_bone
            new.is_direct_channel = True
            new.channel_axis = 'Y'
            new.has_rot = False
            new.has_loc = False
            arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1
            # Initialize result key
            key = f"{PREFIX_RESULT}{_safe_name(selected_bone)}_{_safe_name(name)}"
            try:
                arm.data[key] = 0.0
            except Exception:
                pass
            self.report({'INFO'}, f"Added {name} for {selected_bone}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"记录失败: {ex}")
            return {'CANCELLED'}

class PSDRecordChannelZ(bpy.types.Operator):
    bl_idname = "psd.record_channel_z"
    bl_label = "Record Z Channel"

    def execute(self, context):
        arm = context.object
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}
        selected_bone = _get_selected_pair_bone(context)
        if selected_bone == '<NONE>':
            self.report({'ERROR'}, "骨骼过滤器中未选择骨骼。请先在骨骼过滤器列表中添加/选择一个骨骼。")
            return {'CANCELLED'}
        name = "record_Z"
        # Check if exists
        for existing in arm.psd_saved_poses:
            if existing.bone_name == selected_bone and existing.name == name:
                self.report({'ERROR'}, f"骨骼 {selected_bone} 的条目 '{name}' 已存在")
                return {'CANCELLED'}
        try:
            new = arm.psd_saved_poses.add()
            new.name = name
            new.bone_name = selected_bone
            new.is_direct_channel = True
            new.channel_axis = 'Z'
            new.has_rot = False
            new.has_loc = False
            arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1
            # Initialize result key
            key = f"{PREFIX_RESULT}{_safe_name(selected_bone)}_{_safe_name(name)}"
            try:
                arm.data[key] = 0.0
            except Exception:
                pass
            self.report({'INFO'}, f"Added {name} for {selected_bone}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"记录失败: {ex}")
            return {'CANCELLED'}

classes = [
    PSDSavedPose, PSDBonePair, PSDBonePairUIList, PSD_OT_AddBonePair, PSD_OT_RemoveBonePair, PSD_OT_MoveBonePair,
    PSDStartOperator, PSDStopOperator,
    PSDExportConfig, PSDImportConfig,
    PSDCaptureRest, PSDCaptureRotation, PSDCaptureLocationRest, PSDCaptureLocation,
    PSDSaveCapturedRotationEntry, PSDSaveCapturedLocationEntry, PSDRemoveSavedEntry,
    PSDSavedPoseUIList, PSDPanel, PSDRecordChannelX, PSDRecordChannelY, PSDRecordChannelZ,
    PSDBoneTrigger,
    PSDBoneTriggerUIList,
    PSD_OT_AddTrigger,
    PSD_OT_RemoveTrigger,
    PSD_OT_SelectTriggerBone,
]

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    

    # 触发器
    bpy.types.Object.psd_triggers = CollectionProperty(type=PSDBoneTrigger)
    bpy.types.Object.psd_trigger_index = IntProperty(default=-1)


    # 场景临时存储 (旋转)
    bpy.types.Scene.psd_temp_rest = bpy.props.FloatVectorProperty(size=3, default=(0.0,0.0,0.0))
    bpy.types.Scene.psd_temp_pose = bpy.props.FloatVectorProperty(size=3, default=(0.0,0.0,0.0))
    bpy.types.Scene.psd_temp_rest_bone = bpy.props.StringProperty(default='')
    bpy.types.Scene.psd_temp_pose_bone = bpy.props.StringProperty(default='')

    # 场景临时存储 (位移)
    bpy.types.Scene.psd_temp_loc_rest = bpy.props.FloatVectorProperty(size=3, default=(0.0,0.0,0.0))
    bpy.types.Scene.psd_temp_loc = bpy.props.FloatVectorProperty(size=3, default=(0.0,0.0,0.0))
    bpy.types.Scene.psd_temp_loc_rest_bone = bpy.props.StringProperty(default='')
    bpy.types.Scene.psd_temp_loc_bone = bpy.props.StringProperty(default='')

    bpy.types.Scene.psd_running = bpy.props.BoolProperty(default=False)

    bpy.types.Scene.psd_mode = bpy.props.EnumProperty(
        name="PSD 模式",
        description="AUTO(无实用价值(beta)): 根据播放状态自动切换; FORCE_PLAY(按动画播放器速率): 始终视为播放状态; FORCE_TIMER(推荐): 始终使用计时器采样",
        items=[
            ('AUTO', "自动 (检测)", "根据播放状态自动在播放/计时器模式间切换"),
            ('FORCE_PLAY', "强制播放", "始终视为播放状态 (使用最终依赖图处理器)"),
            ('FORCE_TIMER', "强制计时器", "始终使用计时器采样"),
        ],
        default='AUTO'
    )

    bpy.types.Scene.psd_idle_hz = bpy.props.IntProperty(
        name="空闲频率(Hz)",
        description="非播放状态下计时器更新的频率(Hz) (1..240)",
        default=10,
        min=1,
        max=240
    )

    # 性能调试属性
    bpy.types.Scene.psd_perf_enabled = bpy.props.BoolProperty(
        name="启用性能调试",
        description="显示运行时性能指标 (调试用)",
        default=False
    )
    bpy.types.Scene.psd_perf_history_len = bpy.props.IntProperty(
        name="性能历史",
        description="用于计算每个结果平均延迟的近期样本数",
        default=10,
        min=1,
        max=200
    )
    
    bpy.types.Scene.psd_show_results = bpy.props.BoolProperty(
        name="显示 PSD 结果",
        description="在面板中显示存储在 armature.data 中的所有 PSD 结果（打开可能会影响 UI 性能）",
        default=False
    )
    
        # 搜索 / 排序 / 显示上限（用于 PSD 结果面板）
    bpy.types.Scene.psd_results_search = bpy.props.StringProperty(
        name="搜索 PSD 结果",
        description="按 key 或短名搜索 PSD 结果（大小写不敏感）",
        default=""
    )
    bpy.types.Scene.psd_results_sort_by = bpy.props.EnumProperty(
        name="排序方式",
        description="对 PSD 结果进行排序",
        items=[
            ('NAME', "名字", "按名字排序（短名）"),
        ],
        default='NAME'
    )
    bpy.types.Scene.psd_results_sort_reverse = bpy.props.BoolProperty(
        name="倒序",
        description="倒序排序（开 -> 从大到小）",
        default=False
    )
    bpy.types.Scene.psd_results_limit = bpy.props.IntProperty(
        name="显示上限",
        description="一次最多显示多少条结果（避免 UI 过多）",
        default=200,
        min=1,
        max=5000
    )



    # 逐骨架保存的条目
    bpy.types.Object.psd_saved_poses = bpy.props.CollectionProperty(type=PSDSavedPose)
    bpy.types.Object.psd_saved_pose_index = bpy.props.IntProperty(default=-1)

    # 逐骨架的骨骼过滤器对
    bpy.types.Object.psd_bone_pairs = bpy.props.CollectionProperty(type=PSDBonePair)
    bpy.types.Object.psd_bone_pairs_index = bpy.props.IntProperty(default=0)

def unregister():
    try:
        _remove_handlers_with_name(handlers.depsgraph_update_post, psd_depsgraph_handler.__name__)
    except Exception:
        pass
    try:
        _remove_handlers_with_name(handlers.frame_change_post, psd_frame_handler.__name__)
    except Exception:
        pass

    global _psd_timer_registered, _msgbus_subscribed
    _psd_timer_registered = False
    _unsubscribe_msgbus()

    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except Exception:
            pass

    for p in ('psd_temp_rest','psd_temp_pose','psd_temp_rest_bone','psd_temp_pose_bone',
              'psd_temp_loc_rest','psd_temp_loc','psd_temp_loc_rest_bone','psd_temp_loc_bone',
              'psd_running','psd_mode','psd_idle_hz','psd_perf_enabled','psd_perf_history_len','psd_show_results','psd_results_search','psd_results_sort_by','psd_results_sort_reverse','psd_results_limit'):
        try:
            delattr(bpy.types.Scene, p)
        except Exception:
            pass

    try:
        delattr(bpy.types.Object, 'psd_saved_poses')
        delattr(bpy.types.Object, 'psd_saved_pose_index')
    except Exception:
        pass
    try:
        delattr(bpy.types.Object, 'psd_bone_pairs')
        delattr(bpy.types.Object, 'psd_bone_pairs_index')
    except Exception:
        pass

if __name__ == '__main__':
    register()
