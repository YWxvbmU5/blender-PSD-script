import bpy
import bpy.app.handlers as handlers
from .handlers import _unsubscribe_msgbus, _remove_handlers_with_name, psd_depsgraph_handler, psd_frame_handler


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
    # 没什么用
    # rot_channel_mode: bpy.props.EnumProperty(
    #     name="旋转通道模式",
    #     description="使用摇摆+扭转分解来计算权重（若非 'NONE' 且未启用锥形衰减时生效）",
    #     items=[
    #         ('NONE', "默认", "不使用通道模式 (保持原先的向量投影法)") ,
    #         ('SWING_X_TWIST', "only X 扭转", "将 X 轴作为扭转轴"),
    #         ('SWING_Y_TWIST', "only Y 扭转", "将 Y 轴作为扭转轴"),
    #         ('SWING_Z_TWIST', "only Z 扭转", "将 Z 轴作为扭转轴"),
    #     ],
    #     default='NONE'
    # )

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

    # 缩放通道
    rest_sca: bpy.props.FloatVectorProperty(name="静止缩放", size=3, default=(1.0,1.0,1.0))
    pose_sca: bpy.props.FloatVectorProperty(name="姿态缩放", size=3, default=(1.0,1.0,1.0))
    has_sca: bpy.props.BoolProperty(name="包含缩放", default=False)

    is_direct_channel: bpy.props.BoolProperty(name="Direct Channel Record", default=False)
    channel_axis: bpy.props.EnumProperty(
        name="Channel Axis",
        items=[('X', "X", ""), ('Y', "Y", ""), ('Z', "Z", "")],
        default='X'
    )

class PSDBonePair(bpy.types.PropertyGroup):
    bone_name: bpy.props.StringProperty(name="骨骼", default="")

class PSDBoneTrigger(bpy.types.PropertyGroup):
    """单个触发器条目：放在触发骨骼头周围一个半径（球形），当目标骨骼头进入范围时产生权重"""
    name: bpy.props.StringProperty(name="Name", default="Trigger")
    bone_name: bpy.props.StringProperty(name="Trigger Bone", default="")     # 触发器骨骼（创建时默认活动骨骼）
    target_bone: bpy.props.StringProperty(name="Target Bone", default="")    # 被检测的目标骨骼
    enabled: bpy.props.BoolProperty(name="Enabled", default=True)
    radius: bpy.props.FloatProperty(name="Radius", default=0.2, min=0.0, description="Trigger radius (world units)")
    # 可选：是否线性/平滑衰减（enum），当前仅用线性
    falloff: bpy.props.EnumProperty(
        name="Falloff",
        items=[
            ('LINEAR', "Linear", "Linear falloff (1 - d / r)"),
            ('SMOOTH', "Smoothstep", "Smoothstep falloff"),
        ],
        default='LINEAR'
    )
    # 运行时结果（只读供 UI 显示），不会被序列化为复杂对象，但会保存为小数
    last_weight: bpy.props.FloatProperty(name="Last Weight", default=0.0)

class PSDShapeDriverFile(bpy.types.PropertyGroup):
    filepath: bpy.props.StringProperty(
        name="JSON File",
        subtype='FILE_PATH',
        description="Shape Driver JSON 文件路径"
    )

class PSDPoseDriverFile(bpy.types.PropertyGroup):
    filepath: bpy.props.StringProperty(
        name="JSON File",
        subtype='FILE_PATH',
        description="Pose Driver JSON 文件路径"
    )

def register_props():
    # 注册属性到 bpy.types.Object 和 bpy.types.Scene
    bpy.types.Object.psd_saved_poses = bpy.props.CollectionProperty(type=PSDSavedPose)
    bpy.types.Object.psd_saved_pose_index = bpy.props.IntProperty(default=-1)
    bpy.types.Object.psd_bone_pairs = bpy.props.CollectionProperty(type=PSDBonePair)
    bpy.types.Object.psd_bone_pairs_index = bpy.props.IntProperty(default=0)
    bpy.types.Object.psd_triggers = bpy.props.CollectionProperty(type=PSDBoneTrigger)
    bpy.types.Object.psd_trigger_index = bpy.props.IntProperty(default=-1)
    
    # 场景属性（临时存储等）
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

    #
    bpy.types.Scene.psd_temp_sca_rest = bpy.props.FloatVectorProperty(size=3, default=(1.0,1.0,1.0))
    bpy.types.Scene.psd_temp_sca = bpy.props.FloatVectorProperty(size=3, default=(1.0,1.0,1.0))
    bpy.types.Scene.psd_temp_sca_rest_bone = bpy.props.StringProperty(default='')
    bpy.types.Scene.psd_temp_sca_bone = bpy.props.StringProperty(default='')

    #UI
    bpy.types.Scene.psd_show_captures = bpy.props.BoolProperty(name="显示捕捉数据", default=True)
    bpy.types.Scene.psd_show_triggers = bpy.props.BoolProperty(name="显示触发器", default=True)
    bpy.types.Scene.psd_show_saved_poses = bpy.props.BoolProperty(name="显示已保存姿态", default=True)

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

    bpy.types.Object.psd_output_mode = bpy.props.EnumProperty(
        name="PSD 输出模式",
        description="选择 PSD 计算结果的处理方式",
        items=[
            ('STORE_TO_EMPTY', "存储到 Empty", "将 PSD 结果存储到注册的 Empty（同时应用 Drivers）"),
            ('APPLY_DRIVERS', "仅应用 Drivers", "只根据 JSON Drivers 将结果应用到模型（不存储原始结果）"),
        ],
        default='STORE_TO_EMPTY'
    )

# === 新增：全局（Scene）Shape Driver 和 Pose Driver JSON 文件列表 ===
    bpy.types.Object.psd_shape_driver_files = bpy.props.CollectionProperty(type=PSDShapeDriverFile)
    bpy.types.Object.psd_shape_driver_files_index = bpy.props.IntProperty(default=-1)

    bpy.types.Object.psd_pose_driver_files = bpy.props.CollectionProperty(type=PSDPoseDriverFile)
    bpy.types.Object.psd_pose_driver_files_index = bpy.props.IntProperty(default=-1)

def unregister_props():
    # 删除属性（反向操作）
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


    for p in ('psd_temp_rest','psd_temp_pose','psd_temp_rest_bone','psd_temp_pose_bone',
              'psd_temp_loc_rest','psd_temp_loc','psd_temp_loc_rest_bone','psd_temp_loc_bone',
              'psd_running','psd_mode','psd_idle_hz','psd_perf_enabled','psd_perf_history_len','psd_show_results','psd_results_search','psd_results_sort_by','psd_results_sort_reverse','psd_results_limit',
              'psd_temp_sca_rest','psd_temp_sca','psd_temp_sca_rest_bone','psd_temp_sca_bone','psd_show_captures','psd_show_triggers','psd_show_saved_poses','PSD_OT_invalidate_cache',
              "psd_output_mode","psd_shape_driver_files","psd_shape_driver_files_index",
              "psd_pose_driver_files","psd_pose_driver_files_index"):
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