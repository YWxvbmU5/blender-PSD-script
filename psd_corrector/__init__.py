bl_info = {
    "name": "PSD校正器",
    "author": "s0lus",
    "version": (2, 8),
    "blender": (4, 3, 2),
    "location": "3D视图 > 侧边栏 > PSD",
    "description": "捕捉骨骼的静止/姿态旋转和位置，并将其另存为逐骨架的已保存姿态。PSD算法为每个已保存的姿态（旋转+可选位移）计算单个浮点值，并将其写入空物体数据或者使用JSON。",
    "category": "动画",
}

import bpy
import os
from bpy.props import BoolProperty, StringProperty, FloatProperty, IntProperty
from . import props
from . import utils
from . import math_utils
from . import core
from . import operators
from . import ui
from . import handlers

def _ensure_scene_props():
    # 这些属性名来自你插件的 UI：确保它们都已注册到 bpy.types.Scene
    if not hasattr(bpy.types.Scene, "psd_show_captures"):
        bpy.types.Scene.psd_show_captures = BoolProperty(
            name="PSD Show Captures", description="显示/隐藏 PSD 捕捉数据", default=False
        )
    if not hasattr(bpy.types.Scene, "psd_show_triggers"):
        bpy.types.Scene.psd_show_triggers = BoolProperty(
            name="PSD Show Triggers", description="显示/隐藏 PSD 触发器", default=False
        )

    # 下面是 psd_expr_ui.py 中用到的属性
    if not hasattr(bpy.types.Scene, "psd_expr_mapping_path"):
        bpy.types.Scene.psd_expr_mapping_path = StringProperty(
            name="PSD Mapping File", subtype='FILE_PATH', default=""
        )
    if not hasattr(bpy.types.Scene, "psd_expr_workers"):
        bpy.types.Scene.psd_expr_workers = IntProperty(
            name="Workers", default=max(1, (os.cpu_count() or 2) - 1), min=1, soft_max=64
        )
    if not hasattr(bpy.types.Scene, "psd_expr_chunk_size"):
        bpy.types.Scene.psd_expr_chunk_size = IntProperty(
            name="Chunk Size", default=64, min=1, soft_max=1024
        )
    if not hasattr(bpy.types.Scene, "psd_expr_threshold"):
        bpy.types.Scene.psd_expr_threshold = FloatProperty(
            name="Threshold", default=1e-6, precision=6, min=0.0
        )
    if not hasattr(bpy.types.Scene, "psd_expr_running"):
        bpy.types.Scene.psd_expr_running = BoolProperty(
            name="Expr Running", default=False
        )

def _del_scene_props():
    for prop in ("psd_show_captures", "psd_show_triggers",
                 "psd_expr_mapping_path", "psd_expr_workers",
                 "psd_expr_chunk_size", "psd_expr_threshold",
                 "psd_expr_running"):
        if hasattr(bpy.types.Scene, prop):
            try:
                delattr(bpy.types.Scene, prop)
            except Exception:
                try:
                    del bpy.types.Scene.__dict__[prop]
                except Exception:
                    pass

classes = (
    props.PSDSavedPose,
    props.PSDBonePair,
    props.PSDBoneTrigger,
    props.PSDShapeDriverFile,
    props.PSDPoseDriverFile,
    ui.PSDBonePairUIList,
    ui.PSDSavedPoseUIList,
    ui.PSDBoneTriggerUIList,
    ui.PSDPanel,
    ui.PSDShapeDriverFileUIList,
    ui.PSDPoseDriverFileUIList,
    operators.PSDAddShapeDriverFile,
    operators.PSDRemoveShapeDriverFile,
    operators.PSDAddPoseDriverFile,
    operators.PSDRemovePoseDriverFile, 
    operators.PSDReloadShapeDrivers,
    operators.PSDReloadPoseDrivers,
    operators.PSD_OT_AddBonePair,
    operators.PSD_OT_RemoveBonePair,
    operators.PSD_OT_MoveBonePair,
    operators.PSDExportConfig,
    operators.PSDImportConfig,
    operators.PSDCaptureRest,
    operators.PSDCaptureRotation,
    operators.PSDCaptureLocationRest,
    operators.PSDCaptureLocation,
    operators.PSDCaptureScaleRest,
    operators.PSDCaptureScale,
    operators.PSDSaveCapturedRotationEntry,
    operators.PSDSaveCapturedLocationEntry,
    operators.PSDSaveCapturedScaleEntry,
    operators.PSDRemoveSavedEntry,
    operators.PSD_OT_AddTrigger,
    operators.PSD_OT_RemoveTrigger,
    operators.PSD_OT_SelectTriggerBone,
    operators.PSDRecordChannelX,
    operators.PSDRecordChannelY,
    operators.PSDRecordChannelZ,
    operators.PSD_OT_invalidate_cache,
    operators.PSD_OT_register_cache_empty_ui,
    operators.PSD_OT_unregister_cache_empty_ui,
    handlers.PSDStartOperator,
    handlers.PSDStopOperator,
)

def register():
    #_ensure_scene_props()

    bpy.types.Scene.psd_cache_empty = bpy.props.PointerProperty(
        name="PSD Cache Empty",
        description="选择用于 PSD 缓存的 Empty（必须是 Empty 类型）",
        type=bpy.types.Object
    )

    for cls in classes:
        bpy.utils.register_class(cls)

    # 在 register() 中
    #if not hasattr(bpy.types.Object, "psd_mapping_path"):
    #    bpy.types.Object.psd_mapping_path = StringProperty(name="PSD Mapping File", subtype='FILE_PATH', default="")
    #if not hasattr(bpy.types.Object, "psd_mapping_threshold"):
    #    bpy.types.Object.psd_mapping_threshold = FloatProperty(name="PSD Mapping Threshold", default=1e-4, min=0.0, max=1.0)

    #psd_regmapp.register_mapping_sidebar()
    
    #psd_expr_ui.register_expr_ui()

    # 注册属性（从 props.py 调用）
    props.register_props()
    
    # 注册处理器（从 handlers.py 调用）
    handlers.register_handlers()
    
    # 其他初始化（如核心变量，如果需要）
    core.init_globals()  # 如果 core.py 有初始化函数

def unregister():
    #_del_scene_props()
    #if hasattr(bpy.types.Object, "psd_mapping_path"):
    #    try: del bpy.types.Object.psd_mapping_path
    #    except Exception: pass
    #if hasattr(bpy.types.Object, "psd_mapping_threshold"):
    #    try: del bpy.types.Object.psd_mapping_threshold
    #    except Exception: pass

    #psd_regmapp.unregister_mapping_sidebar()

    #psd_expr_ui.unregister_expr_ui()
    del bpy.types.Scene.psd_cache_empty
    # 注销处理器（从 handlers.py 调用）
    handlers.unregister_handlers()
    
    # 注销属性（从 props.py 调用）
    props.unregister_props()
    
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()