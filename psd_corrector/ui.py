import bpy
from .core import _psd_perf_stats  # 导入性能统计
from .utils import _get_selected_pair_bone  # 导入辅助
from .operators import PREFIX_RESULT, PREFIX_RESULT_LOC, PREFIX_RESULT_SCA

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
            if getattr(item, 'has_sca', False):
                tags.append("S")
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

        # 骨骼过滤器分组
        box = layout.box()
        box.label(text="骨骼过滤器 (若非空，仅计算这些骨骼)", icon='BONE_DATA')
        row = box.row(align=True)
        row.template_list("PSDBonePairUIList", "psd_bone_pairs", arm, "psd_bone_pairs", arm, "psd_bone_pairs_index", rows=4)
        col = row.column(align=True)
        col.operator('psd.add_bone_pair', icon='ADD', text='')
        col.operator('psd.remove_bone_pair', icon='REMOVE', text='')
        col.separator()
        col.operator('psd.move_bone_pair', icon='TRIA_UP', text='').direction = 'UP'
        col.operator('psd.move_bone_pair', icon='TRIA_DOWN', text='').direction = 'DOWN'

        # 配置导出/导入
        box = layout.box()
        box.label(text="配置管理", icon='FILE')
        row = box.row(align=True)
        row.operator('psd.export_config', icon='EXPORT', text='导出')
        row.operator('psd.import_config', icon='IMPORT', text='导入')

        # 显示已选骨骼
        selected_bone = _get_selected_pair_bone(context)
        layout.label(text=f"已选骨骼: {selected_bone}", icon='INFO')

        # 捕捉按钮分组
        box = layout.box()
        box.label(text="捕捉姿态", icon='CAMERA_DATA')
        row = box.row(align=True)
        row.operator('psd.capture_rest', icon='REW', text='静止旋转')
        row.operator('psd.capture_rotation', icon='POSE_HLT', text='姿态旋转')
        row = box.row(align=True)
        row.operator('psd.capture_loc_rest', icon='EMPTY_AXIS', text='静止位置')
        row.operator('psd.capture_location', icon='EMPTY_DATA', text='姿态位置')
        row = box.row(align=True)
        row.operator('psd.capture_sca_rest', icon='EMPTY_AXIS', text='静止缩放')
        row.operator('psd.capture_scale', icon='EMPTY_DATA', text='姿态缩放')

        # 临时捕捉数据（collapsible）
        box = layout.box()
        row = box.row(align=True)
        row.prop(scene, "psd_show_captures", icon='ZOOM_IN' if scene.psd_show_captures else 'ZOOM_OUT', text="捕捉数据")
        if scene.psd_show_captures:
            if scene.psd_temp_pose_bone:
                p = scene.psd_temp_pose
                box.label(text=f"旋转 (姿态): {scene.psd_temp_pose_bone} (X={p[0]:.2f}°, Y={p[1]:.2f}°, Z={p[2]:.2f}°)")
            else:
                box.label(text='无旋转捕捉')
            if scene.psd_temp_rest_bone:
                r = scene.psd_temp_rest
                box.label(text=f"旋转 (静止): {scene.psd_temp_rest_bone} (X={r[0]:.2f}°, Y={r[1]:.2f}°, Z={r[2]:.2f}°)")
            if scene.psd_temp_loc_bone:
                l = scene.psd_temp_loc
                box.label(text=f"位置 (姿态): {scene.psd_temp_loc_bone} (X={l[0]:.4f}, Y={l[1]:.4f}, Z={l[2]:.4f})")
            else:
                box.label(text='无位置捕捉')
            if scene.psd_temp_loc_rest_bone:
                lr = scene.psd_temp_loc_rest
                box.label(text=f"位置 (静止): {scene.psd_temp_loc_rest_bone} (X={lr[0]:.4f}, Y={lr[1]:.4f}, Z={lr[2]:.4f})")
            if scene.psd_temp_sca_bone:
                s = scene.psd_temp_sca
                box.label(text=f"缩放 (姿态): {scene.psd_temp_sca_bone} (X={s[0]:.4f}, Y={s[1]:.4f}, Z={s[2]:.4f})")
            else:
                box.label(text='无缩放捕捉')
            if scene.psd_temp_sca_rest_bone:
                sr = scene.psd_temp_sca_rest
                box.label(text=f"缩放 (静止): {scene.psd_temp_sca_rest_bone} (X={sr[0]:.4f}, Y={sr[1]:.4f}, Z={sr[2]:.4f})")

        # 骨骼触发器（collapsible）
        box = layout.box()
        row = box.row(align=True)
        row.prop(scene, "psd_show_triggers", icon='ZOOM_IN' if scene.psd_show_triggers else 'ZOOM_OUT', text="骨骼触发器")
        if scene.psd_show_triggers:
            subrow = box.row()
            subrow.template_list("PSDBoneTriggerUIList", "psd_triggers", arm, "psd_triggers", arm, "psd_trigger_index", rows=4)
            col = subrow.column(align=True)
            col.operator("psd.add_trigger", icon='ADD', text="")
            col.operator("psd.remove_trigger", icon='REMOVE', text="")
            col.separator()
            col.operator("psd.select_trigger_bone", icon='EYEDROPPER', text="").mode = 'TRIGGER'
            col.operator("psd.select_trigger_bone", icon='RESTRICT_SELECT_OFF', text="").mode = 'TARGET'

            # 选中触发器详情
            idx = arm.psd_trigger_index if arm.psd_trigger_index >= 0 and arm.psd_trigger_index < len(arm.psd_triggers) else -1
            if idx != -1:
                trig = arm.psd_triggers[idx]
                box.prop(trig, "enabled")
                box.prop(trig, "name")
                box.label(text=f"触发骨骼: {trig.bone_name}")
                box.prop(trig, "target_bone", text="目标骨骼")
                box.prop(trig, "radius")
                box.prop(trig, "falloff")
                roww = box.row()
                roww.label(text=f"实时权重: {trig.last_weight:.3f}")

        # 保存按钮分组
        box = layout.box()
        box.label(text="保存条目", icon='FILE_TICK')
        row = box.row(align=True)
        row.operator('psd.save_captured_rotation', text='旋转')
        row.operator('psd.save_captured_location', text='位置')
        row.operator('psd.save_captured_scale', text='缩放')
        row = box.row(align=True)
        row.operator('psd.record_channel_x', text="Record X")
        row.operator('psd.record_channel_y', text="Record Y")
        row.operator('psd.record_channel_z', text="Record Z")

        # 已保存姿态（collapsible）
        box = layout.box()
        row = box.row(align=True)
        row.prop(scene, "psd_show_saved_poses", icon='ZOOM_IN' if scene.psd_show_saved_poses else 'ZOOM_OUT', text="已保存姿态")
        if scene.psd_show_saved_poses:
            bone_name = selected_bone if selected_bone != '<NONE>' else '无'
            subbox = box.box()
            subbox.label(text=f'骨骼 {bone_name} 的姿态条目:', icon='PRESET')
            subrow = subbox.row()
            subrow.template_list('PSDSavedPoseUIList', 'psd_saved_poses', arm, 'psd_saved_poses', arm, 'psd_saved_pose_index')
            col = subrow.column(align=True)
            col.operator('psd.remove_saved_entry', icon='REMOVE', text='')

            # 选中条目详情
            if arm.psd_saved_poses and arm.psd_saved_pose_index >= 0 and arm.psd_saved_pose_index < len(arm.psd_saved_poses):
                e = arm.psd_saved_poses[arm.psd_saved_pose_index]
                if e.bone_name == selected_bone or selected_bone == '<NONE>':
                    subbox.prop(e, 'name', text="名称")
                    subbox.label(text=f"骨骼: {e.bone_name}")

                    # 旋转详情
                    if getattr(e, 'is_direct_channel', False):
                        subbox.label(text=f"Direct Channel: {e.channel_axis}")
                        subbox.prop(e, 'record_rot_channel_mode', text='通道模式')
                    else:
                        subbox.label(text=f"静止旋转: X={e.rest_rot[0]:.2f}°, Y={e.rest_rot[1]:.2f}°, Z={e.rest_rot[2]:.2f}°")
                        subbox.label(text=f"姿态旋转: X={e.pose_rot[0]:.2f}°, Y={e.pose_rot[1]:.2f}°, Z={e.pose_rot[2]:.2f}°")
                        subbox.prop(e, 'has_rot', text='包含旋转')
                        subbox.prop(e, 'cone_enabled', text='锥形衰减')
                        if e.cone_enabled:
                            subbox.prop(e, 'cone_angle', text='锥角 (度)')
                            subbox.prop(e, 'cone_axis', text='轴向')
                        subbox.prop(e, 'rot_channel_mode', text='旋转通道模式')

                    subbox.separator()
                    # 位置详情
                    subbox.label(text=f"静止位置: X={e.rest_loc[0]:.4f}, Y={e.rest_loc[1]:.4f}, Z={e.rest_loc[2]:.4f}")
                    subbox.label(text=f"姿态位置: X={e.pose_loc[0]:.4f}, Y={e.pose_loc[1]:.4f}, Z={e.pose_loc[2]:.4f}")
                    subbox.prop(e, 'has_loc', text='包含位置')
                    subbox.prop(e, 'loc_enabled', text='轴向衰减')
                    if e.loc_enabled:
                        subbox.prop(e, 'loc_radius', text='半径')

                    subbox.separator()
                    # 缩放详情
                    subbox.label(text=f"静止缩放: X={e.rest_sca[0]:.4f}, Y={e.rest_sca[1]:.4f}, Z={e.rest_sca[2]:.4f}")
                    subbox.label(text=f"姿态缩放: X={e.pose_sca[0]:.4f}, Y={e.pose_sca[1]:.4f}, Z={e.pose_sca[2]:.4f}")
                    subbox.prop(e, 'has_sca', text='包含缩放')

        # PSD 结果显示（保持原 collapsible，但用 box 包装）
        box = layout.box()
        box.label(text='PSD 结果 (旋转/位置/缩放)', icon='GRAPH')
        box.prop(scene, "psd_show_results", text="展开显示")
        if scene.psd_show_results:
            subbox = box.box()
            row = subbox.row(align=True)
            row.prop(scene, "psd_results_search", text="", icon='VIEWZOOM')
            row = subbox.row(align=True)
            row.prop(scene, "psd_results_sort_reverse", text="倒序")
            row.prop(scene, "psd_results_limit", text="上限")

            # 原结果收集/显示代码（不变）
            # 收集所有结果到列表（安全地把 keys() 转为 list）
            results = []
            try:
                a_stats = _psd_perf_stats.get(arm.name, {}) if isinstance(_psd_perf_stats, dict) else {}
                entries_stats = a_stats.get("entries", {}) if isinstance(a_stats, dict) else {}
                for k in list(arm.data.keys()):
                    if isinstance(k, str) and (k.startswith(PREFIX_RESULT) or k.startswith(PREFIX_RESULT_LOC) or k.startswith(PREFIX_RESULT_SCA)):
                        # 计算短名与 display 标识
                        if k.startswith(PREFIX_RESULT):
                            short = k[len(PREFIX_RESULT):]
                        elif k.startswith(PREFIX_RESULT_LOC):
                            short = k[len(PREFIX_RESULT_LOC):] + " (位移)"
                        elif k.startswith(PREFIX_RESULT_SCA):
                            short = k[len(PREFIX_RESULT_SCA):] + " (缩放)"
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

        # 模式和调试分组
        box = layout.box()
        box.label(text="运行模式 & 调试", icon='SETTINGS')
        box.prop(scene, "psd_mode", text="模式")
        if scene.psd_mode in ('AUTO', 'FORCE_TIMER'):
            box.prop(scene, "psd_idle_hz", text="空闲 Hz", slider=True)
        box.prop(scene, "psd_perf_enabled", text="性能调试")
        if scene.psd_perf_enabled:
            box.prop(scene, "psd_perf_history_len", text="历史长度")
            # 原性能显示代码（不变）
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

        # 启动/停止按钮
        row = layout.row(align=True)
        if scene.psd_running:
            row.operator('object.psd_stop', icon='CANCEL', text='停止')
        else:
            row.operator('object.psd_start', icon='PLAY', text='启动')

        layout.label(text='(姿态数据保存在骨架上；结果写入骨架数据块)', icon='INFO')
        layout.operator("psd.invalidate_bone_cache", icon='FILE_REFRESH', text='清缓存')
