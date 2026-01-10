import bpy
import math
import json
from .caches import _psd_math_cache, _psd_math_dep_cache

class PoseDriver:
    """
    处理 Pose Driver（后处理表达式 → 骨骼约束属性）的计算与应用。
    """
    def __init__(self, drivers):
        """
        :param drivers: POSE_DRIVERS（已预编译的 dict）
        """
        self.drivers = drivers

    def process(self, arm, arm_key, mem_cache):
        """
        主处理函数，原代码中 Pose Driver 部分完整迁移至此。
        :param arm: 当前 armature 对象
        :param arm_key: armature 的唯一 key
        :param mem_cache: 当前 armature 的 PSD 结果缓存 dict
        """
        if not self.drivers:
            return

        # 使用独立的缓存 key，避免与 Shape Driver 冲突
        pose_cache_key = str(arm_key) + "_pose"
        pose_math_cache = _psd_math_cache.setdefault(pose_cache_key, {})
        pose_dep_snapshot = _psd_math_dep_cache.setdefault(pose_cache_key, {})

        recalculated_pose = 0

        safe_globals = {"__builtins__": {}}
        math_funcs = {
            "cos": math.cos, "sin": math.sin, "tan": math.tan,
            "pi": math.pi, "pow": pow, "abs": abs,
            "max": max, "min": min,
        }

        # 第一步：计算所有 Pose Driver 权重（每个 property 独立）
        for bone_name, constraints in self.drivers.items():
            pb = arm.pose.bones.get(bone_name)
            if not pb:
                continue

            for constraint_name, prop_dict in constraints.items():
                for prop_name, info in prop_dict.items():
                    if info["Armature_name"] != arm.name:
                        continue

                    compiled = info["compiled"]
                    variables = info["variables"]
                    dep_keys = info["dep_keys"]

                    current_input = tuple(mem_cache.get(k, 0.0) for k in dep_keys)
                    cache_key = f"{bone_name}.{constraint_name}.{prop_name}"
                    last_input = pose_dep_snapshot.get(cache_key)

                    if last_input is not None and current_input == last_input:
                        continue

                    local_vars = math_funcs.copy()
                    missing_var = False
                    for var in variables:
                        try:
                            var_key = json.loads(var["data_path"])[0]
                            local_vars[var["name"]] = float(mem_cache.get(var_key, 0.0))
                        except Exception:
                            missing_var = True

                    if missing_var:
                        w = 0.0
                    else:
                        try:
                            w = eval(compiled, safe_globals, local_vars)
                            w = max(0.0, min(1.0, float(w)))
                        except Exception as e:
                            print(f"[Pose Driver] 计算错误 {bone_name}.{constraint_name}.{prop_name}: {e}")
                            w = 0.0

                    pose_math_cache[cache_key] = w
                    pose_dep_snapshot[cache_key] = current_input
                    recalculated_pose += 1

        # 第二步：应用到骨骼约束属性
        if pose_math_cache:
            applied_pose = 0
            for bone_name, constraints in self.drivers.items():
                pb = arm.pose.bones.get(bone_name)
                if not pb:
                    continue

                for constraint_name, prop_dict in constraints.items():
                    constraint = pb.constraints.get(constraint_name)
                    if not constraint:
                        continue

                    for prop_name, info in prop_dict.items():
                        if info["Armature_name"] != arm.name:
                            continue

                        cache_key = f"{bone_name}.{constraint_name}.{prop_name}"
                        w = pose_math_cache.get(cache_key, 0.0)

                        try:
                            prev = getattr(constraint, prop_name, None)
                            if prev is None or abs(prev - w) > 0.001:
                                setattr(constraint, prop_name, w)
                                applied_pose += 1
                        except AttributeError:
                            try:
                                constraint[prop_name] = w
                                applied_pose += 1
                            except Exception:
                                print(f"[Pose Driver] 设置失败 {bone_name}.{constraint_name}.{prop_name}")
                        except Exception as e:
                            print(f"[Pose Driver] 设置错误 {bone_name}.{constraint_name}.{prop_name}: {e}")

            if applied_pose > 0 or recalculated_pose > 0:
                print(f"[Pose Driver] 重算 {recalculated_pose} 条，应用 {applied_pose} 个骨骼约束属性\n")