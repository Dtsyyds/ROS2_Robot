"""
局部坐标系计算模块
为路径点计算局部坐标系（x-扫查方向，y-步进方向，z-法向）
"""
import numpy as np
from typing import List, Dict, Optional


def rotation_matrix_to_euler_angles(R, order='xyz'):
        """
        将旋转矩阵转换为欧拉角
        
        Args:
            R: 3x3 旋转矩阵
            order: 旋转顺序，默认 'xyz' (绕x, y, z轴旋转)
            
        Returns:
            euler_angles: [roll, pitch, yaw] 欧拉角（弧度）
        """
        R = np.array(R, dtype=np.float64)
        
        # 确保是正交矩阵
        assert R.shape == (3, 3), "旋转矩阵必须是 3x3"
        
        if order == 'xyz':
            # XYZ 旋转顺序
            # 提取 pitch (绕 y 轴)
            pitch = np.arcsin(-R[2, 0])
            
            # 检查万向节锁
            if np.abs(np.cos(pitch)) > 1e-6:
                roll = np.arctan2(R[2, 1], R[2, 2])
                yaw = np.arctan2(R[1, 0], R[0, 0])
            else:
                # 万向节锁情况
                roll = 0
                yaw = np.arctan2(-R[0, 1], R[1, 1])
                
        elif order == 'zyx':
            # ZYX 旋转顺序 (常用于航空航天)
            yaw = np.arctan2(R[1, 0], R[0, 0])
            pitch = np.arcsin(-R[2, 0])
            roll = np.arctan2(R[2, 1], R[2, 2])
        else:
            raise ValueError(f"不支持的旋转顺序: {order}")
        
        return np.array([roll, pitch, yaw])

def euler_angles_to_rotation_matrix(angles, order='xyz'):
        """
        将欧拉角转换为旋转矩阵
        
        Args:
            angles: [roll, pitch, yaw] 欧拉角（弧度）
            order: 旋转顺序，默认 'xyz'
            
        Returns:
            R: 3x3 旋转矩阵
        """
        roll, pitch, yaw = angles
        
        # 构造各轴的旋转矩阵
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        if order == 'xyz':
            R = Rz @ Ry @ Rx
        elif order == 'zyx':
            R = Rx @ Ry @ Rz
        else:
            raise ValueError(f"不支持的旋转顺序: {order}")
        
        return R

class LocalCoordinateCalculator:
    """局部坐标系计算器"""
    
    def __init__(self, method='uniform'):
        """
        初始化计算器
        
        Args:
            method: 计算方法 ('uniform'-统一方向，'alternate'-奇偶交替，'weighted'-加权混合，'jiaquan'-加权混合改进版)
        """
        self.method = method
    
    def compute(self, scan_points_3d, scan_normals, scan_orig_indices=None,
                alpha=0.0):
        """
        计算局部坐标系（主入口）

        Args:
            scan_points_3d: 3D 路径点数组 (N, 3)
            scan_normals: 法向量数组 (N, 3)
            scan_orig_indices: 段索引数组
            alpha: 加权参数（仅 weighted/jiaquan 方法使用）

        Returns:
            local_frames: 局部坐标系列表
        """
        if self.method == 'uniform':
            return self.compute_uniform(scan_points_3d, scan_normals, scan_orig_indices)
        elif self.method == 'alternate':
            return self.compute_alternate(scan_points_3d, scan_normals, scan_orig_indices)
        elif self.method == 'weighted':
            return self.compute_weighted(scan_points_3d, scan_normals, scan_orig_indices, alpha)
        elif self.method == 'jiaquan':
            return self.compute_local_frames_jiaquan(scan_points_3d, scan_normals, scan_orig_indices, alpha)
        else:
            raise ValueError(f"未知的方法：{self.method}")
    
    def compute_uniform(self, scan_points_3d, scan_normals, scan_orig_indices=None):
        """
        统一方向法：所有段的 x 轴保持一致方向
        
        规则：
          - 找到第一个偶数段的切线方向作为全局参考
          - 所有点都使用该参考方向（投影到切平面）
        """
        if scan_points_3d is None or len(scan_points_3d) == 0:
            print("⚠️ 路径点为空，无法计算局部坐标系")
            return []

        N = len(scan_points_3d)
        points = np.array(scan_points_3d, dtype=np.float32)

        # 归一化法向量
        normals = np.array(scan_normals, dtype=np.float32)
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = np.divide(normals, norms, out=np.zeros_like(normals), where=(norms != 0))

        # 第一步：计算原始的切线方向（基于路径点序列）
        raw_tangents = np.zeros_like(points)
        for i in range(N):
            if i < N - 1:
                raw_tangents[i] = points[i + 1] - points[i]
            else:
                if N > 1:
                    raw_tangents[i] = points[i] - points[i - 1]
                else:
                    raw_tangents[i] = np.array([1.0, 0.0, 0.0])

        # 归一化原始切线
        raw_tangent_norms = np.linalg.norm(raw_tangents, axis=1, keepdims=True)
        raw_tangents = np.divide(raw_tangents, raw_tangent_norms,
                                 out=np.zeros_like(raw_tangents),
                                 where=(raw_tangent_norms != 0))

        # 第二步：确定参考方向（使用第一个偶数段的切线方向）
        reference_tangent = None
        if scan_orig_indices is not None:
            for i in range(N):
                if scan_orig_indices[i] % 2 == 0:  # 偶数段
                    reference_tangent = raw_tangents[i]
                    break

        # 如果没有段信息，使用第一个点的切线方向
        if reference_tangent is None:
            reference_tangent = raw_tangents[0]

        # 确保参考方向不为零
        if np.linalg.norm(reference_tangent) < 1e-6:
            reference_tangent = np.array([1.0, 0.0, 0.0])

        print(f"参考切线方向：{reference_tangent}")

        # 第三步：计算每个点的最终 x 轴方向
        # 所有点都使用参考方向（保持一致性），然后投影到切平面
        uniform_tangents = np.tile(reference_tangent, (N, 1))

        # 投影到法向量正交平面（确保 x ⊥ z）
        dot_xz = np.sum(uniform_tangents * normals, axis=1, keepdims=True)
        x_axes = uniform_tangents - dot_xz * normals

        # 归一化 x 轴
        x_norms = np.linalg.norm(x_axes, axis=1, keepdims=True)

        # 处理平行情况（x 轴与法向量平行）
        fallback_mask = (x_norms.squeeze() < 1e-6)
        if np.any(fallback_mask):
            print(f"⚠️ {np.sum(fallback_mask)} 个点的参考方向与法向量平行")
            fallback_vec = np.array([1.0, 0.0, 0.0], dtype=np.float32)

            for i in np.where(fallback_mask)[0]:
                z = normals[i]

                # 尝试找到一个与法向量不正交的向量
                candidates = [np.array([1.0, 0.0, 0.0]),
                              np.array([0.0, 1.0, 0.0]),
                              np.array([0.0, 0.0, 1.0])]

                best_candidate = None
                best_dot = 0

                for cand in candidates:
                    proj = cand - np.dot(cand, z) * z
                    proj_norm = np.linalg.norm(proj)
                    if proj_norm > 1e-6:
                        # 检查与参考方向的一致性
                        if np.dot(proj / proj_norm, reference_tangent) > best_dot:
                            best_dot = np.dot(proj / proj_norm, reference_tangent)
                            best_candidate = proj / proj_norm

                if best_candidate is not None:
                    x_axes[i] = best_candidate
                else:
                    # 最后手段：使用交叉积
                    x_axes[i] = np.cross(z, [0, 0, 1])
                    x_norm_temp = np.linalg.norm(x_axes[i])
                    if x_norm_temp > 1e-6:
                        x_axes[i] = x_axes[i] / x_norm_temp
                    else:
                        x_axes[i] = np.cross(z, [1, 0, 0]) / np.linalg.norm(np.cross(z, [1, 0, 0]))

        # 重新归一化所有 x 轴
        x_norms = np.linalg.norm(x_axes, axis=1, keepdims=True)
        x_axes = np.divide(x_axes, x_norms, out=np.zeros_like(x_axes), where=(x_norms != 0))

        # 第四步：计算 y 轴 = z × x （右手系）
        y_axes = np.cross(normals, x_axes)
        y_norms = np.linalg.norm(y_axes, axis=1, keepdims=True)
        y_axes = np.divide(y_axes, y_norms, out=np.zeros_like(y_axes), where=(y_norms != 0))

        # 存储结果
        local_frames = []
        for i in range(N):
            frame = {
                'origin': points[i].tolist(),
                'x_axis': x_axes[i].tolist(),
                'y_axis': y_axes[i].tolist(),
                'z_axis': normals[i].tolist()
            }

            # 记录原始段信息（如果可用）
            if scan_orig_indices is not None:
                frame['orig_idx'] = int(scan_orig_indices[i])
                frame['is_odd_segment'] = bool(scan_orig_indices[i] % 2 == 1)

            local_frames.append(frame)

        print(f"✅ 已计算 {N} 个路径点的局部坐标系（所有段保持相同 x 轴方向）")
        return local_frames
    
    def compute_alternate(self, scan_points_3d, scan_normals, scan_orig_indices=None):
        """
        奇偶段交替反向法
        
        规则：
          - 段 0（第一个偶数段）：原始方向
          - 段 1、3、5...（奇数段）：与段 0 同向
          - 段 2、4、6...（偶数段）：交替反向
        """
        if scan_points_3d is None or len(scan_points_3d) == 0:
            print("⚠️ 路径点为空，无法计算局部坐标系")
            return []

        N = len(scan_points_3d)
        points = np.array(scan_points_3d, dtype=np.float32)

        # 归一化法向量
        normals = np.array(scan_normals, dtype=np.float32)
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = np.divide(normals, norms, out=np.zeros_like(normals), where=(norms != 0))

        # 步骤 1：计算每个点的原始切线方向
        raw_tangents = np.zeros_like(points)
        for i in range(N):
            if i < N - 1:
                raw_tangents[i] = points[i + 1] - points[i]
            elif i > 0:
                raw_tangents[i] = points[i] - points[i - 1]
            else:
                raw_tangents[i] = np.array([1.0, 0.0, 0.0])

        # 归一化原始切线
        raw_tangent_norms = np.linalg.norm(raw_tangents, axis=1, keepdims=True)
        raw_tangents = np.divide(raw_tangents, raw_tangent_norms,
                                 out=np.zeros_like(raw_tangents),
                                 where=(raw_tangent_norms != 0))

        # 步骤 2：计算段 0 的平均原始切线方向（作为奇数段的参考方向）
        first_even_avg_direction = None
        if scan_orig_indices is not None:
            segment0_indices = [i for i in range(N) if scan_orig_indices[i] == 0]
            if segment0_indices:
                sum_direction = np.zeros(3)
                for idx in segment0_indices:
                    sum_direction += raw_tangents[idx]
                avg_direction = sum_direction / len(segment0_indices)
                norm = np.linalg.norm(avg_direction)
                if norm > 1e-6:
                    first_even_avg_direction = avg_direction / norm

        if first_even_avg_direction is None:
            first_even_avg_direction = raw_tangents[0]

        # 步骤 3：为每个点分配调整后的切线方向
        adjusted_tangents = np.zeros_like(raw_tangents)
        for i in range(N):
            if scan_orig_indices is not None and i < len(scan_orig_indices):
                orig_idx = scan_orig_indices[i]
                if orig_idx % 2 == 1:  # 奇数段
                    # 奇数段使用第一个偶数段的方向
                    adjusted_tangents[i] = first_even_avg_direction
                else:  # 偶数段
                    # 偶数段：交替反向
                    factor = 1.0 if (orig_idx // 2) % 2 == 0 else -1.0
                    adjusted_tangents[i] = raw_tangents[i] * factor
            else:
                adjusted_tangents[i] = raw_tangents[i]

        # 步骤 4：投影到法向量切平面，得到 x 轴
        dot_xz = np.sum(adjusted_tangents * normals, axis=1, keepdims=True)
        x_axes = adjusted_tangents - dot_xz * normals

        # 归一化 x 轴
        x_norms = np.linalg.norm(x_axes, axis=1, keepdims=True)

        # 处理与法向量平行的情况
        fallback_mask = (x_norms.squeeze() < 1e-6)
        if np.any(fallback_mask):
            print(f"⚠️ {np.sum(fallback_mask)} 个点的切线与法向量平行，使用备选方向")
            for i in np.where(fallback_mask)[0]:
                z = normals[i]
                x_axes[i] = np.cross(z, [0, 0, 1])
                x_norm_temp = np.linalg.norm(x_axes[i])
                if x_norm_temp > 1e-6:
                    x_axes[i] = x_axes[i] / x_norm_temp
                else:
                    x_axes[i] = np.cross(z, [1, 0, 0]) / np.linalg.norm(np.cross(z, [1, 0, 0]))

        # 重新归一化
        x_norms = np.linalg.norm(x_axes, axis=1, keepdims=True)
        x_axes = np.divide(x_axes, x_norms, out=np.zeros_like(x_axes), where=(x_norms != 0))

        # 计算 y 轴 = z × x
        y_axes = np.cross(normals, x_axes)
        y_norms = np.linalg.norm(y_axes, axis=1, keepdims=True)
        y_axes = np.divide(y_axes, y_norms, out=np.zeros_like(y_axes), where=(y_norms != 0))

        # 存储结果
        local_frames = []
        for i in range(N):
            frame = {
                'origin': points[i].tolist(),
                'x_axis': x_axes[i].tolist(),
                'y_axis': y_axes[i].tolist(),
                'z_axis': normals[i].tolist()
            }
            if scan_orig_indices is not None:
                frame['orig_idx'] = int(scan_orig_indices[i])
                frame['is_odd_segment'] = bool(scan_orig_indices[i] % 2 == 1)
            local_frames.append(frame)

        print(f"✅ 已计算 {N} 个路径点的局部坐标系（奇偶段交替反向）")
        return local_frames
    
    def compute_weighted(self, scan_points_3d, scan_normals, scan_orig_indices=None, alpha=0.0):
        """
        加权混合法：在局部切线和全局方向之间加权
        
        Args:
            alpha: 加权参数 (0.0=纯局部，1.0=纯全局)
        """
        if scan_points_3d is None or len(scan_points_3d) == 0:
            print("⚠️ 路径点为空，无法计算局部坐标系")
            return []

        N = len(scan_points_3d)
        points = np.array(scan_points_3d, dtype=np.float32)

        # 归一化法向量
        normals = np.array(scan_normals, dtype=np.float32)
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = np.divide(normals, norms, out=np.zeros_like(normals), where=(norms != 0))

        # 步骤 1：计算原始切线
        raw_tangents = np.zeros_like(points)
        for i in range(N):
            if i < N - 1:
                raw_tangents[i] = points[i + 1] - points[i]
            elif i > 0:
                raw_tangents[i] = points[i] - points[i - 1]
            else:
                raw_tangents[i] = np.array([1.0, 0.0, 0.0])

        raw_tangent_norms = np.linalg.norm(raw_tangents, axis=1, keepdims=True)
        raw_tangents = np.divide(raw_tangents, raw_tangent_norms,
                                 out=np.zeros_like(raw_tangents),
                                 where=(raw_tangent_norms != 0))

        # 步骤 2：确定全局参考方向
        reference_tangent = None
        if scan_orig_indices is not None:
            for i in range(N):
                if scan_orig_indices[i] % 2 == 0:
                    reference_tangent = raw_tangents[i]
                    break
        if reference_tangent is None:
            reference_tangent = raw_tangents[0]

        # 步骤 3：加权混合
        adjusted_tangents = np.zeros_like(raw_tangents)
        for i in range(N):
            if scan_orig_indices is not None and i < len(scan_orig_indices):
                orig_idx = scan_orig_indices[i]
                if orig_idx % 2 == 1:  # 奇数段
                    # 奇数段：全局参考方向权重更高
                    adjusted_tangents[i] = (1 - alpha) * raw_tangents[i] + alpha * reference_tangent
                else:  # 偶数段
                    factor = 1.0 if (orig_idx // 2) % 2 == 0 else -1.0
                    adjusted_tangents[i] = ((1 - alpha) * raw_tangents[i] + alpha * reference_tangent) * factor
            else:
                adjusted_tangents[i] = raw_tangents[i]

        # 归一化混合后的切线
        adj_norms = np.linalg.norm(adjusted_tangents, axis=1, keepdims=True)
        adjusted_tangents = np.divide(adjusted_tangents, adj_norms,
                                      out=np.zeros_like(adjusted_tangents),
                                      where=(adj_norms != 0))

        # 步骤 4：投影到切平面
        dot_xz = np.sum(adjusted_tangents * normals, axis=1, keepdims=True)
        x_axes = adjusted_tangents - dot_xz * normals

        x_norms = np.linalg.norm(x_axes, axis=1, keepdims=True)
        fallback_mask = (x_norms.squeeze() < 1e-6)
        if np.any(fallback_mask):
            for i in np.where(fallback_mask)[0]:
                z = normals[i]
                x_axes[i] = np.cross(z, [0, 0, 1])
                x_norm_temp = np.linalg.norm(x_axes[i])
                if x_norm_temp > 1e-6:
                    x_axes[i] = x_axes[i] / x_norm_temp
                else:
                    x_axes[i] = np.cross(z, [1, 0, 0]) / np.linalg.norm(np.cross(z, [1, 0, 0]))

        x_norms = np.linalg.norm(x_axes, axis=1, keepdims=True)
        x_axes = np.divide(x_axes, x_norms, out=np.zeros_like(x_axes), where=(x_norms != 0))

        # 计算 y 轴
        y_axes = np.cross(normals, x_axes)
        y_norms = np.linalg.norm(y_axes, axis=1, keepdims=True)
        y_axes = np.divide(y_axes, y_norms, out=np.zeros_like(y_axes), where=(y_norms != 0))

        # 存储结果
        local_frames = []
        for i in range(N):
            frame = {
                'origin': points[i].tolist(),
                'x_axis': x_axes[i].tolist(),
                'y_axis': y_axes[i].tolist(),
                'z_axis': normals[i].tolist()
            }
            if scan_orig_indices is not None:
                frame['orig_idx'] = int(scan_orig_indices[i])
            local_frames.append(frame)

        print(f"---✅ 已计算 {N} 个路径点的局部坐标系（加权混合 alpha={alpha}）---")
        return local_frames

    def compute_local_frames_jiaquan(self, scan_points_3d, scan_normals, scan_orig_indices=None, alpha=0.0):
        """
        为每个路径点计算局部坐标系（加权版）：
          - z: 法向量（归一化）
          - x: (1-α) * 局部投影切线 + α * 全局参考投影方向，再归一化 & 符号对齐
          - y: z × x （右手系）

        Args:
            scan_points_3d: 3D 路径点数组 (N, 3)
            scan_normals: 法向量数组 (N, 3)
            scan_orig_indices: 段索引数组
            alpha (float): 权重 ∈ [0, 1]
                - 0.0: 完全信任局部路径走向（等价于上一版）
                - 1.0: 完全强制统一方向（类似你最原始版本）
                - 0.2~0.5: 推荐值，平衡自然性与一致性

        """
        if scan_points_3d is None or len(scan_points_3d) == 0:
            print("⚠️ 路径点为空，无法计算局部坐标系")
            return []

        assert 0.0 <= alpha <= 1.0, "alpha must be in [0, 1]"

        N = len(scan_points_3d)
        points = np.array(scan_points_3d, dtype=np.float32)
        normals = np.array(scan_normals, dtype=np.float32)

        # === Step 1: 归一化法向量 ===
        norms = np.linalg.norm(normals, axis=1, keepdims=True)
        normals = np.divide(normals, norms, out=np.zeros_like(normals), where=(norms != 0))

        # === Step 2: 计算并投影局部切线 ===
        raw_tangents = np.zeros_like(points)
        if N == 1:
            raw_tangents[0] = np.array([1.0, 0.0, 0.0])
        else:
            raw_tangents[0] = points[1] - points[0]
            if(raw_tangents[0][0]<0):
                raw_tangents[0][0]=-raw_tangents[0][0]
            raw_tangents[-1] = points[-1] - points[-2]
            for i in range(1, N - 1):
                raw_tangents[i] = points[i + 1] - points[i - 1]
            tan_norms = np.linalg.norm(raw_tangents, axis=1, keepdims=True)
            raw_tangents = np.divide(raw_tangents, tan_norms,
                                     out=np.zeros_like(raw_tangents),
                                     where=(tan_norms != 0))
        # for i in range(0, 10):     
        #     print(f"raw_tangents[{i}]:")
        #     print(raw_tangents[i])
        #     print(f"points{i}:")
        #     print(points[i])
            
        # 投影到切平面 → 局部自然方向
        dot_tz = np.sum(raw_tangents * normals, axis=1, keepdims=True)
        local_x_dir = raw_tangents - dot_tz * normals
        # print("local_x_dir[0]:", local_x_dir[0])
        local_x_norms = np.linalg.norm(local_x_dir, axis=1, keepdims=True)
        local_x_dir = np.divide(local_x_dir, local_x_norms,
                                out=np.zeros_like(local_x_dir),
                                where=(local_x_norms > 1e-8))

        # === Step 3: 构建全局参考投影方向 ===
        # 参考方向：第一个偶数段（或首个有效点）的投影切线
        reference_tangent = None
        if scan_orig_indices is not None:
            for i, idx in enumerate(scan_orig_indices):
                if idx % 2 == 0 and np.linalg.norm(local_x_dir[i]) > 1e-6:
                    reference_tangent = local_x_dir[i+1].copy()
                    # [    0.99999  0.00011112  -0.0040693]
                    # [   -0.99948   -0.032236  -0.0015641]
                    if(reference_tangent[0]<0):
                        reference_tangent[0]=-reference_tangent[0]
                    break
        if reference_tangent is None:
            for i in range(N):
                if np.linalg.norm(local_x_dir[i]) > 1e-6:
                    reference_tangent = local_x_dir[i].copy()
                    break
        if reference_tangent is None or np.linalg.norm(reference_tangent) < 1e-6:
            reference_tangent = np.array([1.0, 0.0, 0.0])

        # 将参考方向投影到每个点的切平面（使其 ⊥ z_i）
        dot_rz = np.sum(reference_tangent * normals, axis=1, keepdims=True)
        global_x_dir = reference_tangent - dot_rz * normals  # shape: (N, 3)
        global_x_norms = np.linalg.norm(global_x_dir, axis=1, keepdims=True)
        global_x_dir = np.divide(global_x_dir, global_x_norms,
                                 out=np.zeros_like(global_x_dir),
                                 where=(global_x_norms > 1e-8))

        # === Step 4: 【加权混合】 x = (1-α) * local + α * global ===
        mixed_x = (1.0 - alpha) * local_x_dir + alpha * global_x_dir

        # === Step 5: 符号对齐（防止因加权导致整体翻转）===
        # 用参考方向检查整体朝向
        mixed_x_norms = np.linalg.norm(mixed_x, axis=1, keepdims=True)
        mixed_x = np.divide(mixed_x, mixed_x_norms,
                            out=np.zeros_like(mixed_x),
                            where=(mixed_x_norms > 1e-8))

        dot_with_ref = np.sum(mixed_x * reference_tangent, axis=1)
        flip_mask = dot_with_ref < 0
        x_axes = mixed_x.copy()
        x_axes[flip_mask] *= -1.0

        # === Step 6: 处理退化（norm ≈ 0）===
        x_norms = np.linalg.norm(x_axes, axis=1, keepdims=True)
        degenerate_mask = (x_norms.squeeze() < 1e-6)
        if np.any(degenerate_mask):
            print(f"⚠️ {np.sum(degenerate_mask)} 个点需 fallback（加权后退化）")
            for i in np.where(degenerate_mask)[0]:
                z = normals[i]
                # 优先尝试 local_x_dir（若有效）
                if np.linalg.norm(local_x_dir[i]) > 1e-6:
                    x_axes[i] = local_x_dir[i] if np.dot(local_x_dir[i], reference_tangent) >= 0 else -local_x_dir[i]
                else:
                    # fallback 到参考方向投影
                    proj = reference_tangent - np.dot(reference_tangent, z) * z
                    if np.linalg.norm(proj) > 1e-6:
                        x_axes[i] = proj / np.linalg.norm(proj)
                        if np.dot(x_axes[i], reference_tangent) < 0:
                            x_axes[i] *= -1
                    else:
                        cand = np.cross(z, [0, 0, 1])
                        if np.linalg.norm(cand) < 1e-6:
                            cand = np.cross(z, [1, 0, 0])
                        x_axes[i] = cand / np.linalg.norm(cand)

        # 最终归一化
        x_norms = np.linalg.norm(x_axes, axis=1, keepdims=True)
        x_axes = np.divide(x_axes, x_norms, out=np.zeros_like(x_axes), where=(x_norms > 1e-8))

        # === Step 7: y = z × x ===
        y_axes = np.cross(normals, x_axes)
        y_norms = np.linalg.norm(y_axes, axis=1, keepdims=True)
        y_axes = np.divide(y_axes, y_norms, out=np.zeros_like(y_axes), where=(y_norms > 1e-8))

        # === 存储 & 验证 ===
        local_frames = []
        for i in range(N):
            frame = {
                'origin': points[i].tolist(),
                'x_axis': x_axes[i].tolist(),
                'y_axis': y_axes[i].tolist(),
                'z_axis': normals[i].tolist()
            }
            if scan_orig_indices is not None:
                orig_idx = int(scan_orig_indices[i])
                frame['orig_idx'] = orig_idx
                frame['is_odd_segment'] = bool(orig_idx % 2 == 1)
            local_frames.append(frame)

        # 验证
        if local_frames:
            even_x = odd_x = None
            for frame in local_frames:
                if 'orig_idx' in frame:
                    x_vec = np.array(frame['x_axis'])
                    if even_x is None and frame['orig_idx'] % 2 == 0:
                        even_x = x_vec
                    if odd_x is None and frame['orig_idx'] % 2 == 1:
                        odd_x = x_vec
                if even_x is not None and odd_x is not None:
                    dot_prod = np.dot(even_x, odd_x)
                    print(f"alpha={alpha:.2f} → 偶/奇段x点积: {dot_prod:.4f}")
                    if dot_prod < 0.95:
                        print("⚠️ 方向一致性不足（可增大 alpha）")
                    break

        print(f"✅ 已计算 {N} 个路径点的局部坐标系（alpha={alpha}，加权混合）")
        return local_frames

    def frames_to_scan_points(self, local_frames):
        """
        将局部坐标系转换为输出格式 [[x, y, z, x1,x2,x3, y1,y2,y3, nx,ny,nz], ...]
        
        Args:
            local_frames: 局部坐标系列表
            
        Returns:
            scan_points: N×12 的列表
        """
        if not local_frames:
            return []

        scan_points = []
       
        for frame in local_frames:
            o = frame['origin']
                # 使用欧拉角格式: [x, y, z, rx, ry, rz]

                # 构造旋转矩阵（列向量是坐标轴）
            R = np.column_stack([
                    frame['x_axis'],
                    frame['y_axis'],
                    frame['z_axis']
                ])
                # 转换为欧拉角（弧度）
            euler = rotation_matrix_to_euler_angles(R)
            scan_points.append(o + euler.tolist())  # 3+3 = 6
        return scan_points
        