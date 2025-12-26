import bpy
import json
import os
import math
from bpy_extras.io_utils import ImportHelper, ExportHelper
from .utils import _safe_name, _get_selected_pair_bone, _capture_bone_local_rotation_deg, _capture_bone_local_translation, _capture_bone_local_scale  # 导入依赖
from .core import psd_invalidate_bone_cache  # 导入核心函数
from .utils import PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA

class PSDExportConfig(bpy.types.Operator, ExportHelper):
    """导出当前骨架的 PSD 配置（包含每个 bone_pair 对应的所有条目）"""
    bl_idname = "psd.export_config"
    bl_label = "导出 PSD 配置"
    filename_ext = ".json"
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

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
                    "group_name": getattr(e, "group_name", ""),
                    "rest_sca": list(e.rest_sca),
                    "pose_sca": list(e.pose_sca),
                    "has_sca": bool(e.has_sca)
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
    filter_glob: bpy.props.StringProperty(default="*.json", options={'HIDDEN'})

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
            
            #scale
            try:
                rs = ent.get("rest_sca", [1.0, 1.0, 1.0])
                if len(rs) >= 3:
                    new.rest_sca = (float(rs[0]), float(rs[1]), float(rs[2]))
            except Exception:
                pass
            try:
                ps = ent.get("pose_sca", [1.0, 1.0, 1.0])
                if len(ps) >= 3:
                    new.pose_sca = (float(ps[0]), float(ps[1]), float(ps[2]))
            except Exception:
                pass
            try:
                new.has_sca = bool(ent.get("has_sca", False))
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
                         "has_loc","loc_enabled","loc_radius","group_name","rest_sca","pose_sca","has_sca","sca_enabled","sca_radius"}:
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

class PSDCaptureScaleRest(bpy.types.Operator):
    bl_idname = "psd.capture_sca_rest"
    bl_label = "捕捉静止缩放"

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
            rest_sca, _ = _capture_bone_local_scale(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_sca_rest_bone = selected_bone
            context.scene.psd_temp_sca_rest = tuple(rest_sca)
            self.report({'INFO'}, f"已为 {selected_bone} 捕捉静止缩放: {tuple(rest_sca)}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"捕捉静止缩放失败: {ex}")
            return {'CANCELLED'}

class PSDCaptureScale(bpy.types.Operator):
    bl_idname = "psd.capture_scale"
    bl_label = "捕捉姿态缩放"

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
            _, pose_sca = _capture_bone_local_scale(arm, selected_bone, depsgraph=None)
            context.scene.psd_temp_sca_bone = selected_bone
            context.scene.psd_temp_sca = tuple(pose_sca)
            self.report({'INFO'}, f"已为 {selected_bone} 捕捉姿态缩放: {tuple(pose_sca)}")
            return {'FINISHED'}
        except Exception as ex:
            self.report({'ERROR'}, f"捕捉姿态缩放失败: {ex}")
            return {'CANCELLED'}

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

        # 确保缩放字段为空/禁用
        new.rest_sca = (0.0,0.0,0.0)
        new.pose_sca = (0.0,0.0,0.0)
        new.has_sca = False

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
        # 确保缩放字段为空/禁用
        new.rest_sca = (0.0,0.0,0.0)
        new.pose_sca = (0.0,0.0,0.0)
        new.has_sca = False
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

class PSDSaveCapturedScaleEntry(bpy.types.Operator):
    bl_idname = "psd.save_captured_scale"
    bl_label = "保存捕捉的缩放条目"

    entry_name: bpy.props.StringProperty(name="条目名称", default="default")

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        arm = context.object
        scene = context.scene
        if not arm or arm.type != 'ARMATURE':
            self.report({'ERROR'}, "请先选择一个骨架")
            return {'CANCELLED'}

        bone_sca_r = getattr(scene, 'psd_temp_sca_rest_bone', '')
        bone_sca_p = getattr(scene, 'psd_temp_sca_bone', '')

        has_sca = bool(bone_sca_p and bone_sca_p != '<NONE>')
        if not has_sca:
            self.report({'ERROR'}, "请先捕捉一个缩放 (必需)")
            return {'CANCELLED'}

        if bone_sca_r and bone_sca_r != bone_sca_p and bone_sca_r != '':
            self.report({'WARNING'}, "缩放的静止/姿态来自不同骨骼；如有需要，此条目的静止值将设为(1,1,1)")
            rest_vals_sca = (1.0,1.0,1.0)
        else:
            rest_vals_sca = tuple(getattr(scene, 'psd_temp_sca_rest', (1.0,1.0,1.0)))

        pose_vals_sca = tuple(getattr(scene, 'psd_temp_sca', (1.0,1.0,1.0)))
        bone_name_sca = getattr(scene, 'psd_temp_sca_bone', '')

        # 检查该骨骼是否已存在同名条目
        for existing in arm.psd_saved_poses:
            if existing.bone_name == bone_name_sca and existing.name == self.entry_name:
                self.report({'ERROR'}, f"骨骼 {bone_name_sca} 的条目 '{self.entry_name}' 已存在")
                return {'CANCELLED'}

        # 创建条目 (仅缩放)
        new = arm.psd_saved_poses.add()
        new.name = self.entry_name if self.entry_name else "default"
        new.bone_name = bone_name_sca
        new.is_direct_channel = False
        # 旋转和位置字段为空/禁用
        new.rest_rot = (0.0,0.0,0.0)
        new.pose_rot = (0.0,0.0,0.0)
        new.has_rot = False
        new.rest_loc = (0.0,0.0,0.0)
        new.pose_loc = (0.0,0.0,0.0)
        new.has_loc = False
        # 设置缩放字段（无衰减）
        new.rest_sca = rest_vals_sca
        new.pose_sca = pose_vals_sca
        new.has_sca = True

        arm.psd_saved_pose_index = len(arm.psd_saved_poses) - 1

        # 清除临时捕捉数据
        scene.psd_temp_sca_rest = (1.0,1.0,1.0)
        scene.psd_temp_sca = (1.0,1.0,1.0)
        scene.psd_temp_sca_rest_bone = ''
        scene.psd_temp_sca_bone = ''

        # 创建并初始化缩放结果的键
        entry_key_sca = f"{PREFIX_RESULT_SCA}{_safe_name(new.bone_name)}_{_safe_name(new.name)}"
        try:
            arm.data[entry_key_sca] = 0.0
        except Exception:
            pass

        self.report({'INFO'}, f"已在骨架 {arm.name} 上为骨骼 {bone_name_sca} 保存缩放条目 '{new.name}'")
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
            entry_key_sca = f"{PREFIX_RESULT_SCA}{_safe_name(entry.bone_name)}_{_safe_name(entry.name)}"
            
            # 改进1: 显式获取 Armature Datablock，与写入函数保持一致
            arm_db = bpy.data.armatures.get(arm.data.name)

            if arm_db:
                # 从数据块中安全地删除自定义属性
                if entry_key_rot in arm_db:
                    del arm_db[entry_key_rot]
                if entry_key_loc in arm_db:
                    del arm_db[entry_key_loc]
                if entry_key_sca in arm_db:
                    del arm_db[entry_key_sca]
            
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
    mode: bpy.props.EnumProperty(items=[('TRIGGER','Trigger','Set trigger bone'), ('TARGET','Target','Set target bone')], default='TRIGGER')
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

# 其他所有 Operator，如 PSDImportConfig, PSDCaptureRest, PSDSaveCapturedRotationEntry, PSDRemoveSavedEntry, PSD_OT_AddTrigger 等
# ... (完整复制所有 Operator 类，从 "class PSDExportConfig" 到文件末尾的 Operator)

class PSD_OT_invalidate_cache(bpy.types.Operator):
    bl_idname = "psd.invalidate_bone_cache"
    bl_label = "Invalidate PSD Bone Cache"

    def execute(self, context):
        obj = context.object
        if obj and obj.type == 'ARMATURE':
            psd_invalidate_bone_cache(obj.name)
            self.report({'INFO'}, f"Cleared PSD cache for {obj.name}")
        else:
            psd_invalidate_bone_cache()
            self.report({'INFO'}, "Cleared all PSD caches")
        return {'FINISHED'}
