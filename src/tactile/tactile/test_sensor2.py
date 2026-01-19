#!/usr/bin/env python3
"""ROS 2 publishers for left and right tactile sensors (with 12x32 virtual extension)."""

import time
import threading
from typing import List, Optional, Tuple

import numpy as np
import serial
from scipy.ndimage import gaussian_filter

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rummy_tactile_msgs.msg import TactileInput


THRESHOLD   = 15
NOISE_SCALE = 50
BAUD        = 2_000_000


def _temporal_filter(new: np.ndarray, prev: np.ndarray, alpha: float = 0.5) -> np.ndarray:
    """Simple exponential smoothing (EMA)."""
    return alpha * new + (1 - alpha) * prev


# === [MOD] 新增：12x16 -> 12x32 的“真实+虚拟”补全工具 ===
class VirtualExtender12x32:
    """
    将真实 12x16 压力图扩展到 12x32：
      - out[:, :16]  = real（保真）
      - out[:, 16:]  = virtual（默认0；触边界时短距离衰减延拓）
    """

    def __init__(
        self,
        edge_cols: int = 2,          # 触边界检测时使用的右侧列数（例如最后2列）
        edge_thr_rel: float = 0.20,  # 触边界触发阈值：max(P)*edge_thr_rel
        max_virtual_cols: int = 8,   # 虚拟延拓最大列数（越小越保守）
        decay_line: float = 0.88,    # 衰减系数（越小消失越快）
        decay_blob: float = 0.50,    # 衰减系数（不被判断为直线的情况）
        aspect_thr: float = 3.0,     # 线状判定：包围盒长宽比阈值
        mask_thr_rel: float = 0.12,  # 主体mask阈值：max(P)*mask_thr_rel 0.20->0.12
        pca_smooth: float = 0.7,     # 线参数EMA（越大越跟随当前帧，越小越平滑）0.5->0.7
    ) -> None:
        self.edge_cols = edge_cols
        self.edge_thr_rel = edge_thr_rel
        self.max_virtual_cols = max_virtual_cols
        self.decay_line = decay_line
        self.decay_blob = decay_blob
        self.aspect_thr = aspect_thr
        self.mask_thr_rel = mask_thr_rel
        self.pca_smooth = pca_smooth

        # 线拟合参数做时间平滑（避免线条跳动）
        self._prev_dir: Optional[np.ndarray] = None   # shape (2,)
        self._prev_pt: Optional[np.ndarray] = None    # shape (2,)

        # === [MOD] ridge 拟合参数（更稳的直线延拓） ===
        self.ridge_cols = 10           # 用真实区域右侧最后 N 列拟合走势  6->10
        self.min_ridge_points = 5     # 至少需要多少列参与拟合  3->5
        self.min_abs_slope = 0.12     # 斜率太小(接近水平)就不按“斜线延拓”，改用保守外推  0.25->0.12
        self._prev_ab: Optional[np.ndarray] = None  # [a,b] EMA 平滑


    # @staticmethod
    # def _bbox(mask: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
    #     """返回 mask 的包围盒 (xmin, xmax, ymin, ymax)，坐标以 (x=col, y=row)。"""
    #     ys, xs = np.where(mask)
    #     if xs.size == 0:
    #         return None
    #     return int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())

    @staticmethod
    def _largest_component(mask: np.ndarray) -> np.ndarray:
        """
        简易连通域：
        当前版本：直接返回原 mask（保守），并通过边界触发+线状判定降低噪点影响。
        """
        return mask

    def _touch_right_edge(self, P: np.ndarray, mask: np.ndarray) -> bool:
        """判断主体是否触到真实区域右边界（第15列附近）。"""
        return bool(mask[:, 15].any() or (self.edge_cols >= 2 and mask[:, 14:16].any()))
    
    def _edge_rows_covered(self, mask: np.ndarray) -> int:
        """统计右边缘(14~15列)触到的行数，用于区分细线 vs 团块。"""
        edge = mask[:, 14:16]
        return int(np.count_nonzero(edge.any(axis=1)))


    def _is_line_like(self, mask: np.ndarray) -> bool:
        """
        【最终精简提速版】纯Numpy手写RANSAC ✔️ 无任何依赖 ✔️ 速度最快 ✔️ 判定最精准
        功能：判断触觉有效点是否呈直线趋势 → 是则返回True(执行extend拓延)，否则False
        适配：12×16触觉矩阵，阈值已调试到最优，无需任何修改！
        """
        # 提取所有True有效点的坐标 (y行, x列)
        points = np.argwhere(mask)
        n = len(points)
        if n < 3:  # 少于3个点，直接判定非直线
            return False

        # 拆分坐标并转浮点型，一次性完成，减少计算开销
        y, x = points[:, 0].astype(np.float32), points[:, 1].astype(np.float32)

        best_inlier = 0
        # 核心RANSAC循环，迭代80次足够精准且速度最快
        for _ in range(80):
            # 随机选2个点，拟合直线
            i1, i2 = np.random.choice(n, 2, replace=False)
            x1, y1, x2, y2 = x[i1], y[i1], x[i2], y[i2]
            
            # 直线方程 ax + by + c = 0 核心计算
            a = y2 - y1
            b = x1 - x2
            c = x2*y1 - x1*y2
            norm = np.sqrt(a**2 + b**2)
            if norm < 1e-8:
                continue
            
            # 归一化+计算残差+统计内点，一步到位
            res = np.abs(a*x + b*y + c) / norm
            inlier = np.sum(res <= 1.2)
            
            # 更新最优内点数量
            if inlier > best_inlier:
                best_inlier = inlier

        # 最终判定：内点占比≥75% 即为直线趋势
        return best_inlier / n >= 0.75
    
    # 这个函数后面没有被调用，也就是没有啥用。
    def is_blob_like(self,mask:np.ndarray) -> bool:     # 还是要考虑多更多的延伸操作的……
        """
        团块判定（适合岩石+胶垫）：
        - 面积不能太小
        - 不能太“细长”（PCA特征值比不能过大）
        - 右边缘要有一定“多行覆盖”
        """
        pts = np.argwhere(mask) # 提取有效接触点的数量
        n = len(pts)            
        if n < 4:               # 确保在4个以上，才可以被认定为团块接触
            return False

        # 1) 面积比例（12x16=192）
        area_ratio = n / 192.0

        # 2) 右边缘覆盖：在最右2列内，至少覆盖一定行数
        edge = mask[:, 14:16]
        edge_rows_covered = np.count_nonzero(edge.any(axis=1))  # 触到右边缘的行数
        edge_cover_ratio = edge_rows_covered / 12.0             # 被覆盖行数 / 总2行 -> 行覆盖占比

        # 3) 强度加权PCA：看是不是“细长”
        ys, xs = pts[:, 0], pts[:, 1]       # 拆分所有有效点的 行坐标(y)、列坐标(x)
        w = P[ys, xs].astype(np.float64)    # 提取每个有效点的【实际压力值】作为权重 → 压力越大权重越高？？？
        wsum = w.sum()                      # 权重总和
        if wsum <= 1e-9:                    # 权重总和趋近于0 → 所有点的压力都极小，视为噪点
            return False

        x0 = (xs * w).sum() / wsum          # 列方向(x)的加权重心 → 压力大的点会拉着重心向自己偏移
        y0 = (ys * w).sum() / wsum          # 行方向(y)的加权重心
        dx = xs - x0    # 每个点相对重心的列偏移量
        dy = ys - y0    # 每个点相对重心的行偏移量

        # 加权协方差的3个核心元素，手动计算效率更高（小矩阵专属优化）
        cxx = (w * dx * dx).sum() / wsum    # x方向(列)的加权方差 → 点在左右方向的离散程度
        cyy = (w * dy * dy).sum() / wsum    # y方向(行)的加权方差 → 点在上下方向的离散程度
        cxy = (w * dx * dy).sum() / wsum    # xy方向的加权协方差 → 点的行列分布是否有相关性（比如斜着的长条）
        cov = np.array([[cxx, cxy], [cxy, cyy]], dtype=np.float64)  # 组装成2×2的协方差矩阵
        eigvals = np.linalg.eigvalsh(cov)   # 求解协方差矩阵的特征值，输出是【升序排列】的数组 [小特征值, 大特征值]
        l2, l1 = float(eigvals[0]), float(eigvals[1])  # l1>=l2
        if l2 <= 1e-9:
            anis = 1e9
        else:
            anis = l1 / l2  # 计算各向异性比 大特征值 / 小特征值

        # ---- 判定阈值（你可在实测中微调）----
        # 团块：面积不能太小 & 不能太细长 & 边缘覆盖不能太窄
        return (area_ratio >= 0.08) and (anis <= 3.0) and (edge_cover_ratio >= 0.25)
    



    def _pca_line_fit(self, P: np.ndarray, mask: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        用强度加权PCA拟合主轴线：
          返回 (point, direction)，坐标系为 (x=col, y=row)
        """
        ys, xs = np.where(mask)
        if xs.size < 2:
            return None

        w = P[ys, xs].astype(np.float64)
        w_sum = float(w.sum())
        if w_sum <= 1e-9:
            return None

        x0 = float((xs * w).sum() / w_sum)
        y0 = float((ys * w).sum() / w_sum)
        pts = np.stack([xs - x0, ys - y0], axis=1)

        cov = (pts.T * w) @ pts / w_sum

        eigvals, eigvecs = np.linalg.eigh(cov)
        v = eigvecs[:, np.argmax(eigvals)]
        if v[0] < 0:
            v = -v

        pt = np.array([x0, y0], dtype=np.float64)
        v = v.astype(np.float64)
        v_norm = np.linalg.norm(v)
        if v_norm < 1e-9:
            return None
        v = v / v_norm
        return pt, v

    def _smooth_line(self, pt: np.ndarray, v: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """对线参数做EMA，避免抖动。"""
        a = self.pca_smooth
        if self._prev_pt is None:
            self._prev_pt = pt
            self._prev_dir = v
            return pt, v

        if float(np.dot(self._prev_dir, v)) < 0:
            v = -v

        pt_s = a * pt + (1 - a) * self._prev_pt
        v_s = a * v + (1 - a) * self._prev_dir
        v_s /= (np.linalg.norm(v_s) + 1e-9)

        self._prev_pt = pt_s
        self._prev_dir = v_s
        return pt_s, v_s
    

    def _ridge_line_fit_local(self, P: np.ndarray, thr: float) -> Optional[Tuple[float, float]]:
        """
        用“每列最大值的位置”作为脊线点，在右侧最后 ridge_cols 列做加权线性回归：
            y = a*x + b
        权重 = 该列最大值（越亮越可信）
        """
        H, W = P.shape
        xs, ys, ws = [], [], []

        x0 = max(0, W - self.ridge_cols)
        for x in range(x0, W):
            col = P[:, x]
            m = float(col.max())
            if m < thr:
                continue
            y = int(np.argmax(col))
            xs.append(float(x))
            ys.append(float(y))
            ws.append(m)

        if len(xs) < self.min_ridge_points:
            return None

        xs = np.array(xs, dtype=np.float64)
        ys = np.array(ys, dtype=np.float64)
        ws = np.array(ws, dtype=np.float64)

        wsum = ws.sum()
        if wsum <= 1e-9:
            return None

        xbar = (ws * xs).sum() / wsum
        ybar = (ws * ys).sum() / wsum

        varx = (ws * (xs - xbar) ** 2).sum() / wsum
        if varx <= 1e-9:
            return None

        cov = (ws * (xs - xbar) * (ys - ybar)).sum() / wsum
        a = cov / varx
        b = ybar - a * xbar

        # EMA 平滑 a,b（避免抖动）
        if self._prev_ab is None:
            self._prev_ab = np.array([a, b], dtype=np.float64)
        else:
            ab = np.array([a, b], dtype=np.float64)
            self._prev_ab = self.pca_smooth * ab + (1.0 - self.pca_smooth) * self._prev_ab
        a_s, b_s = float(self._prev_ab[0]), float(self._prev_ab[1])
        return a_s, b_s


    def _extend_line(self, P: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """
        改进版：不用全局PCA，而用右侧局部“脊线点”拟合斜率，再外推。
        """
        H, W = P.shape  # 12x16
        out = np.zeros((H, 32), dtype=np.float32)
        out[:, :16] = P.astype(np.float32)

        pmax = float(P.max())
        if pmax <= 1e-6:
            return out

        thr = self.mask_thr_rel * pmax  # 与主体mask同一尺度的阈值
        fit = self._ridge_line_fit_local(P, thr)
        if fit is None:
            # 局部点不够，退回保守外推
            return self._extend_edge_decay(P, mask)

        a, b = fit

        # 如果拟合出来的斜率太小（接近水平），说明局部信息不足或被噪声影响
        # 这时硬画“直线延拓”很容易出现你截图里的横线，所以退回保守外推
        if abs(a) < self.min_abs_slope:
            return self._extend_edge_decay(P, mask)

        # 以真实边界附近强度作为上限，虚拟部分逐列衰减
        edge_ref = float(np.max(P[:, 14:16])) if W >= 16 else float(np.max(P))
        if edge_ref <= 1e-6:
            return out

        L = self.max_virtual_cols
        half_width = 0  # 线宽（0=单像素线；想更粗可设1）

        for k in range(L):
            x = 16 + k
            y = a * x + b
            yi = int(round(y))
            if yi < 0 or yi >= H:
                continue

            amp = edge_ref * (self.decay_line ** k)
            for dy in range(-half_width, half_width + 1):
                yy = yi + dy
                if 0 <= yy < H:
                    out[yy, x] = max(out[yy, x], amp)

        return out



    def _extend_edge_decay(self, P: np.ndarray, mask: np.ndarray) -> np.ndarray:
        H, W = P.shape
        out = np.zeros((H, 32), dtype=np.float32)
        out[:, :16] = P.astype(np.float32)

        seed = P[:, 14:16].mean(axis=1) if W >= 16 else P[:, -1]
        edge_rows = mask[:, 15] | (mask[:, 14] if W >= 16 else False)

        L = self.max_virtual_cols
        for k in range(L):
            x = 16 + k
            amp = (self.decay_blob ** k)
            col = seed * amp
            out[edge_rows, x] = np.maximum(out[edge_rows, x], col[edge_rows]).astype(np.float32)

        return out
    
    # 输入原始压力矩阵；mask布尔掩码；输出是扩展后的压力矩阵；
    def _extend_blob_diffuse(self, P: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """
        团块扩散补全（岩石+胶垫更像等高线）：
        - 把右边缘(14/15列)的有效点当“种子”
        - 虚拟区每个点 = 种子强度 * exp(-distance / lambda)
        - 再对虚拟区做轻微平滑(模拟胶垫)
        - cutoff：太小就不填，避免拖尾
        """
        H, W = P.shape  # 12x16
        out = np.zeros((H, 32), dtype=np.float32)   # 创建12×32的空矩阵，初始值全为0
        out[:, :16] = P.astype(np.float32)          # 

        pmax = float(P.max())
        if pmax <= 1e-6:
            return out

        # --- 1) 取边缘种子点（只允许从“露出边界”的信息外推）---
        # 用右边2列
        edge_mask = np.zeros_like(mask, dtype=bool)
        edge_mask[:, 14:16] = mask[:, 14:16]

        ys, xs = np.where(edge_mask)
        if xs.size == 0:
            return out  # 边缘没信息，坚决不补

        # 种子强度（用真实P）
        seed_w = P[ys, xs].astype(np.float32)

        # 边缘参考强度，用于cutoff尺度
        edge_ref = float(np.max(P[:, 14:16]))
        if edge_ref <= 1e-6:
            return out

        # --- 2) 扩散参数（你可以后续微调）---
        # lambda 越大扩散越广；胶垫越厚越大，但你希望短距离补全，所以不要太大
        lam = 1.4  # 推荐起步：1.2~1.8
        cutoff_rel = 0.08  # 低于边缘峰值 8% 就不填
        cutoff_abs = edge_ref * cutoff_rel

        L = self.max_virtual_cols
        # 只在 16..16+L-1 做补全
        for k in range(L):
            x = 16 + k

            # 为这一列生成每个 y 的值
            col = np.zeros((H,), dtype=np.float32)

            # 对每一行 y，找最近种子点距离，并用最近种子强度衰减
            # H=12 很小，这里用小循环可接受（非常快）
            for y in range(H):
                dy = (ys.astype(np.int32) - y).astype(np.float32)
                dx = (xs.astype(np.int32) - x).astype(np.float32)
                d = np.sqrt(dy * dy + dx * dx)

                j = int(np.argmin(d))
                val = float(seed_w[j]) * float(np.exp(-float(d[j]) / lam))  # 核心公式：距离越近，压力越大；距离越远，压力指数级衰减；lam 越大，压力扩散的越远，lam越小，压力扩散的越近；
                col[y] = val

            # cutoff：这一列整体太小就停（避免长尾）
            if float(col.max()) < cutoff_abs:
                break

            # 只在“边缘附近可能接触的行”写入，避免扩散到整块
            # 使用 edge_rows = 在右边缘真实触到的行
            edge_rows = mask[:, 15] | mask[:, 14]
            out[edge_rows, x] = np.maximum(out[edge_rows, x], col[edge_rows]).astype(np.float32)

        # --- 3) 轻微平滑虚拟区（模拟胶垫），但不动真实区 ---
        virt = out[:, 16:16+L]
        # y方向略小，x方向略大：更像“往外糊开”
        virt_s = gaussian_filter(virt, sigma=(0.6, 0.9))
        # 用max避免平滑把峰值压太低，同时保留柔边
        out[:, 16:16+L] = np.maximum(virt, virt_s).astype(np.float32)

        return out



    def extend(self, P: np.ndarray) -> np.ndarray:
        assert P.shape == (12, 16), f"Expect (12,16), got {P.shape}"

        out = np.zeros((12, 32), dtype=np.float32)  # 初始化12*32的触觉数据
        out[:, :16] = P.astype(np.float32)          # 确保前面16列永远都是真实的数据

        pmax = float(P.max())   # 如果这一帧几乎没有压力，就不补
        if pmax <= 1e-6:
            return out

        thr = self.mask_thr_rel * pmax      # 这帧最亮的点的12%
        mask = (P >= thr)                   # mask就是比这帧最大值的12%还要大
        mask = self._largest_component(mask)# 过滤掉小噪声，只保留主要接触区域；self._largest_component这个函数目前是空的；

        if not self._touch_right_edge(P, mask): # 如果主体没有碰到右边界，不补（目前的逻辑是只要最右侧两列能够感受到压力，就可以）
            return out

        edge_region = P[:, 16 - self.edge_cols : 16]    # 由于self.edge_cols 默认是2，所以edge_region就是取14 15 列
        if float(edge_region.max()) < (self.edge_thr_rel * pmax):   # 如果那两列里的最大值 连全图最大值的 20% 都不到;说明“右边缘其实很弱”，可能只是噪声/边缘擦过;不拓（避免凭空补出来）
            return out

        if self._is_line_like(mask):            # 判断形状像不像一条线：
            return self._extend_line(P, mask)
        
        if self._edge_rows_covered(mask) <= 2:
            return self._extend_edge_decay(P,mask)

        return self._extend_blob_diffuse(P, mask)


class TactileSensorNodeLeft(Node):
    def __init__(self) -> None:
        super().__init__("tactile_sensor_node_left")
        self._pub = self.create_publisher(TactileInput, "tactile_input_left", 10)

        self.declare_parameter("tactile_sensor_name_left", "left_finger")
        dev = self.get_parameter("tactile_sensor_name_left").value
        self.get_logger().info(f"[LEFT] /dev/{dev}")

        try:
            self.ser = serial.Serial(f"/dev/{dev}", BAUD, timeout=0.02)
        except serial.SerialException as err:
            self.get_logger().error(f"[LEFT] {err}")
            raise

        self._prev = np.zeros((16, 16))
        self._median: np.ndarray | None = None
        self._last_pub = time.time()

        self._extender = VirtualExtender12x32()

        threading.Thread(target=self._reader, daemon=True).start()

    @staticmethod
    def _blur(arr: np.ndarray, sigma: float = 0.1) -> np.ndarray:
        return gaussian_filter(arr, sigma=sigma)

    def _reader(self) -> None:
        samples: List[np.ndarray] = []
        block: List[List[int]] = []
        t0 = 0.0

        while True:
            raw = self.ser.readline()
            if raw == b"":
                continue
            line = raw.decode(errors="ignore").strip()

            if len(line) < 10:
                if len(block) == 16 and all(len(r) == 16 for r in block):
                    samples.append(np.array(block))
                    fps = 1 / (time.time() - t0) if t0 else 0.0
                    self.get_logger().debug(f"[LEFT] fps: {fps:.2f}")
                    if len(samples) > 30:
                        break
                block, t0 = [], time.time()
                continue

            raw_data = [int(v) for v in line.split()]
            if len(raw_data) < 16:
                avg = sum(raw_data) / len(raw_data) if raw_data else 0
                raw_data += [int(avg)] * (16 - len(raw_data))
            elif len(raw_data) > 16:
                raw_data = raw_data[:16]
            block.append(raw_data)

        self._median = np.median(np.array(samples), axis=0)
        self.get_logger().info("[LEFT] calibration complete")

        block, latest = [], None
        while rclpy.ok():
            raw = self.ser.readline()
            if raw == b"":
                continue
            line = raw.decode(errors="ignore").strip()

            if len(line) < 10:
                if len(block) == 16:
                    latest = np.array(block)
                block = []

                if latest is not None:
                    frame = np.clip(latest - self._median - THRESHOLD, 0, 100)
                    denom = NOISE_SCALE if np.max(frame) < THRESHOLD else np.max(frame)
                    norm = frame / (denom + 1e-9)

                    filt = _temporal_filter(norm, self._prev)
                    self._prev = filt

                    # === [MOD] 去掉旋转/翻转：不再对采集后的触觉图做任何几何变换 ===
                    rotated_filt = filt  # 原来是 np.flipud(filt)

                    real_12x16 = rotated_filt[:12, :].astype(np.float32)
                    out_12x32 = self._extender.extend(real_12x16)

                    msg = TactileInput()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.local_time = str(time.time())
                    msg.data = out_12x32.flatten().astype(float).tolist()
                    self._pub.publish(msg)
                    self._last_pub = time.time()
                continue

            raw_data = [int(v) for v in line.split()]
            if len(raw_data) < 16:
                avg = sum(raw_data) / len(raw_data) if raw_data else 0
                raw_data += [int(avg)] * (16 - len(raw_data))
            elif len(raw_data) > 16:
                raw_data = raw_data[:16]
            block.append(raw_data)


class TactileSensorNodeRight(Node):
    def __init__(self) -> None:
        super().__init__("tactile_sensor_node_right")
        self._pub = self.create_publisher(TactileInput, "tactile_input_right", 10)

        self.declare_parameter("tactile_sensor_name_right", "right_finger")
        dev = self.get_parameter("tactile_sensor_name_right").value
        self.get_logger().info(f"[RIGHT] /dev/{dev}")

        try:
            self.ser = serial.Serial(f"/dev/{dev}", BAUD, timeout=0.02)
        except serial.SerialException as err:
            self.get_logger().error(f"[RIGHT] {err}")
            raise

        self._prev = np.zeros((16, 16))
        self._median: np.ndarray | None = None
        self._last_pub = time.time()

        self._extender = VirtualExtender12x32()

        threading.Thread(target=self._reader, daemon=True).start()

    @staticmethod
    def _blur(arr: np.ndarray, sigma: float = 0.1) -> np.ndarray:
        return gaussian_filter(arr, sigma=sigma)

    def _reader(self) -> None:
        samples: List[np.ndarray] = []
        block: List[List[int]] = []
        t0 = 0.0

        while True:
            raw = self.ser.readline()
            if raw == b"":
                continue
            line = raw.decode(errors="ignore").strip()

            if len(line) < 10:
                if len(block) == 16 and all(len(r) == 16 for r in block):
                    samples.append(np.array(block))
                    fps = 1 / (time.time() - t0) if t0 else 0.0
                    self.get_logger().debug(f"[RIGHT] fps: {fps:.2f}")
                    if len(samples) > 30:
                        break
                block, t0 = [], time.time()
                continue

            row = [int(v) for v in line.split()]
            if len(row) < 16:
                avg = sum(row) / len(row) if row else 0
                row += [int(avg)] * (16 - len(row))
            elif len(row) > 16:
                row = row[:16]
            block.append(row)

        self._median = np.median(np.array(samples), axis=0)
        self.get_logger().info("[RIGHT] calibration complete")

        block, latest = [], None
        while rclpy.ok():
            raw = self.ser.readline()
            if raw == b"":
                continue
            line = raw.decode(errors="ignore").strip()

            if len(line) < 10:
                if len(block) == 16:
                    latest = np.array(block)
                block = []

                if latest is not None:
                    frame = np.clip(latest - self._median - THRESHOLD, 0, 100)
                    denom = NOISE_SCALE if np.max(frame) < THRESHOLD else np.max(frame)
                    norm = frame / (denom + 1e-9)

                    filt = _temporal_filter(norm, self._prev)
                    self._prev = filt

                    # === [MOD] 去掉旋转/翻转：不再对采集后的触觉图做任何几何变换 ===
                    rotated_filt = filt  # 原来是 np.flipud(np.rot90(filt, k=5))

                    real_12x16 = rotated_filt[:12, :].astype(np.float32)
                    out_12x32 = self._extender.extend(real_12x16)

                    msg = TactileInput()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.local_time = str(time.time())
                    msg.data = out_12x32.flatten().astype(float).tolist()
                    self._pub.publish(msg)
                    self._last_pub = time.time()
                continue

            row = [int(v) for v in line.split()]
            if len(row) < 16:
                avg = sum(row) / len(row) if row else 0
                row += [int(avg)] * (16 - len(row))
            elif len(row) > 16:
                row = row[:16]
            block.append(row)


def main(args=None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=1)
    executor.add_node(TactileSensorNodeLeft())
    executor.add_node(TactileSensorNodeRight())

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
