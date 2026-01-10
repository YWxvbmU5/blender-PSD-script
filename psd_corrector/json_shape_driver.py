import json
import bpy
import math
from .caches import _psd_math_cache, _psd_math_dep_cache

class ShapeDriver:
    """
    处理 Shape Driver（后处理表达式 → Shape Key 值）的计算与应用。
    """
    def __init__(self, expressions):
        """
        :param expressions: POST_PROCESS_EXPRESSIONS（已预编译的 dict）
        """
        self.expressions = expressions
        # 模块级缓存
        self.shape_key_index = {}      # {mesh_name: {shape_name: index}}
        self.shape_sliders = {}        # {mesh_name: (mins_list, maxs_list)}
        self.shape_last_buffer = {}    # {mesh_name: last_values_list}

    def process(self, arm_key, mem_cache):
        """
        :param arm_key: armature 的唯一 key（通常是 as_pointer()）
        :param mem_cache: 当前 armature 的 PSD 结果缓存 dict
        """
        if not self.expressions:
            return

        # 缓存
        math_cache = _psd_math_cache.setdefault(arm_key, {})
        dep_snapshot = _psd_math_dep_cache.setdefault(arm_key, {})

        recalculated = 0

        # 安全数学函数
        safe_globals = {"__builtins__": {}}
        math_funcs = {
            "cos": math.cos, "sin": math.sin, "tan": math.tan,
            "pi": math.pi, "pow": pow, "abs": abs,
            "max": max, "min": min,
        }

        # 第一步：计算所有 Shape Driver 权重
        for post_key, info in self.expressions.items():
            compiled = info["compiled"]
            variables = info["variables"]
            dep_keys = info["dep_keys"]

            current_input = tuple(mem_cache.get(k, 0.0) for k in dep_keys)
            last_input = dep_snapshot.get(post_key)

            if last_input is not None and current_input == last_input:
                continue  # 依赖未变，跳过

            local_vars = math_funcs.copy()
            missing_var = False
            for var in variables:
                try:
                    var_key = json.loads(var["data_path"])[0]
                    value = mem_cache.get(var_key, 0.0)
                    local_vars[var["name"]] = float(value)
                except Exception:
                    missing_var = True

            if missing_var:
                w = 0.0
            else:
                try:
                    w = eval(compiled, safe_globals, local_vars)
                    w = float(w)
                    w = max(0.0, min(1.0, w))
                except Exception as e:
                    print(f"[Shape Driver] 计算错误 {post_key}: {e}")
                    w = 0.0

            math_cache[post_key] = w
            dep_snapshot[post_key] = current_input
            recalculated += 1

        # 第二步：应用到 Shape Keys
        if math_cache:
            mesh_groups = {}
            for post_key, info in self.expressions.items():
                mesh_name = info.get("Mesh_name")
                if mesh_name:
                    mesh_groups.setdefault(mesh_name, []).append(post_key)

            applied_total = 0
            for mesh_name, post_keys in mesh_groups.items():
                obj = bpy.data.objects.get(mesh_name)
                if not obj or obj.type != 'MESH' or not obj.data.shape_keys:
                    print(f"[Shape Driver] 警告：找不到有效 Mesh '{mesh_name}'")
                    continue

                key_blocks = obj.data.shape_keys.key_blocks
                n = len(key_blocks)
                if n == 0:
                    continue

                # 索引缓存
                idx_map = self.shape_key_index.get(mesh_name)
                if idx_map is None or len(idx_map) != n:
                    idx_map = {kb.name: i for i, kb in enumerate(key_blocks)}
                    self.shape_key_index[mesh_name] = idx_map

                # 范围缓存
                sliders = self.shape_sliders.get(mesh_name)
                if sliders is None or len(sliders[0]) != n:
                    mins = [kb.slider_min for kb in key_blocks]
                    maxs = [kb.slider_max for kb in key_blocks]
                    sliders = (mins, maxs)
                    self.shape_sliders[mesh_name] = sliders
                mins, maxs = sliders

                # 值缓冲区（复用上帧）
                buf = self.shape_last_buffer.get(mesh_name)
                if buf is None or len(buf) != n:
                    buf = [0.0] * n

                updated_count = 0
                for post_key in post_keys:
                    w_raw = math_cache.get(post_key, 0.0)
                    idx = idx_map.get(post_key)
                    if idx is not None:
                        v = w_raw
                        buf[idx] = float(v)
                        updated_count += 1

                try:
                    key_blocks.foreach_set("value", buf)
                    self.shape_last_buffer[mesh_name] = buf[:]
                except Exception as e:
                    print(f"[Shape Driver] foreach_set 失败 ({mesh_name}): {e}")
                    # 回退到逐个设置
                    for post_key in post_keys:
                        kb = key_blocks.get(post_key)
                        if kb:
                            w_raw = math_cache.get(post_key, 0.0)
                            kb.value = max(kb.slider_min, min(kb.slider_max, w_raw))

                if updated_count > 0:
                    print(f"[Shape Driver] → Mesh '{mesh_name}' 批量更新 {updated_count}/{len(post_keys)} 个 Shape Key (foreach_set)")
                    applied_total += updated_count

            print(f"[Shape Driver] 应用 {applied_total} 个 Shape Key (总驱动 {len(math_cache)} 条)\n")