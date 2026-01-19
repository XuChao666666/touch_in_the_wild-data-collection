#!/usr/bin/env python3
"""ROS 2 publisher for a single (right) tactile sensor."""

import time
import threading
import json
from pathlib import Path
from typing import List

import numpy as np
import serial

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rummy_tactile_msgs.msg import TactileInput

THRESHOLD   = 15
NOISE_SCALE = 50
BAUD        = 2_000_000


def _temporal_filter(new: np.ndarray, prev: np.ndarray, alpha: float = 0.5) -> np.ndarray:
    return alpha * new + (1 - alpha) * prev


class TactileSensorNodeRight(Node):
    def __init__(self) -> None:
        super().__init__("tactile_sensor_node_right")
        self._pub = self.create_publisher(TactileInput, "tactile_input_right", 10)

        self.declare_parameter("tactile_sensor_name_right", "right_finger")
        dev = self.get_parameter("tactile_sensor_name_right").value
        self.get_logger().info(f"[RIGHT] /dev/{dev}")

        # （可选）输出文件名也做成参数，方便 launch 时改
        self.declare_parameter("calib_dump_path", "tactile_calib_frames_right.json")
        self._calib_dump_path = self.get_parameter("calib_dump_path").value

        try:
            self.ser = serial.Serial(f"/dev/{dev}", BAUD)
        except serial.SerialException as err:
            self.get_logger().error(f"[RIGHT] {err}")
            raise

        self._prev = np.zeros((16, 16))
        self._median: np.ndarray | None = None
        self._last_pub = time.time()

        threading.Thread(target=self._reader, daemon=True).start()

    @staticmethod
    def _parse_numeric_row(line: str) -> list[int] | None:
        parts = line.split()
        if not parts:
            return None
        try:
            return [int(p) for p in parts]
        except ValueError:
            return None


    # def _reader(self) -> None:
    #     # -------- calibration stage --------
    #     samples: List[np.ndarray] = []
    #     block: List[List[int]] = []
    #     calib_blocks: List[List[List[int]]] = []   # 用于保存 31 帧原始 block
    #     t0 = 0.0

    #     while True:
    #         if self.ser.in_waiting == 0:
    #             continue

    #         line = self.ser.readline().decode(errors="ignore").strip()

    #         # frame delimiter / empty-ish line
    #         if len(line) < 10:
    #             if len(block) == 16 and all(len(r) == 16 for r in block):
    #                 samples.append(np.array(block))
    #                 calib_blocks.append([row[:] for row in block])  # 深拷贝保存

    #                 fps = 1 / (time.time() - t0) if t0 else 0.0
    #                 self.get_logger().debug(f"[RIGHT] fps: {fps:.2f}")

    #                 if len(samples) > 30:  # 31 帧
    #                     break

    #             block, t0 = [], time.time()
    #             continue

    #         # block.append([int(v) for v in line.split()])
    #         row = self._parse_numeric_row(line)
    #         if row is None:
    #             self.get_logger().warn(f"[RIGHT] non-numeric line from sensor: {line!r}")
    #             # 如果你希望看到更多上下文，也可以把它当作“帧分隔符”来重置 block：
    #             block = []
    #             continue
            
    #         block.append(row)

    #     # 计算基线
    #     self._median = np.median(np.array(samples), axis=0)

    #     # -------- dump calibration frames to JSON --------
    #     # 生成 {"01": [[...],[...]], ..., "31": ...}
    #     dump_obj = {f"{i+1:02d}": calib_blocks[i] for i in range(len(calib_blocks))}

    #     dump_path = Path(self._calib_dump_path)
    #     try:
    #         dump_path.parent.mkdir(parents=True, exist_ok=True)
    #         with dump_path.open("w", encoding="utf-8") as f:
    #             json.dump(dump_obj, f, ensure_ascii=False, indent=2)
    #         self.get_logger().info(f"[RIGHT] calibration frames saved to: {dump_path}")
    #     except Exception as e:
    #         self.get_logger().error(f"[RIGHT] failed to write calib json: {e}")

    #     self.get_logger().info("[RIGHT] calibration complete")

    #     # -------- running stage --------
    #     block, latest = [], None
    #     while rclpy.ok():
    #         if self.ser.in_waiting == 0:
    #             continue

    #         line = self.ser.readline().decode(errors="ignore").strip()

    #         if len(line) < 10:
    #             if len(block) == 16:
    #                 latest = np.array(block)
    #             block = []

    #             if latest is not None:
    #                 frame = np.clip(latest - self._median - THRESHOLD, 0, 100)
    #                 denom = NOISE_SCALE if np.max(frame) < THRESHOLD else np.max(frame)
    #                 norm  = frame / denom

    #                 filt = _temporal_filter(norm, self._prev)
    #                 self._prev = filt

    #                 rotated_filt = np.flipud(np.rot90(filt, k=5))

    #                 msg = TactileInput()
    #                 msg.header.stamp = self.get_clock().now().to_msg()
    #                 msg.local_time = str(time.time())
    #                 msg.data = rotated_filt[:12].flatten().astype(float).tolist()

    #                 self._pub.publish(msg)
    #                 self._last_pub = time.time()

    #             continue

    #         block.append([int(v) for v in line.split()])


    # def _reader(self) -> None:
    #     # -------- calibration stage (debug, conservative) --------
    #     samples: List[np.ndarray] = []
    #     block: List[List[int]] = []
    #     calib_blocks: List[List[List[int]]] = []
    #     t0 = 0.0

    #     # === debug counters ===
    #     dbg_total = 0
    #     dbg_short = 0
    #     dbg_nonnum = 0
    #     dbg_good16 = 0
    #     dbg_otherlen = 0
    #     dbg_min_len = 10**9
    #     dbg_max_len = 0
    #     dbg_last_report = time.time()

    #     # 保存一些样例，方便你确认格式（不会刷屏）
    #     short_examples: List[str] = []
    #     nonnum_examples: List[str] = []
    #     badlen_examples: List[str] = []

    #     # 如果超过这个阈值还没看到短行，就明确报错提示（但不改变行为）
    #     warn_after_lines = 500
    #     warn_after_seconds = 5.0
    #     start_time = time.time()
    #     warned_no_short = False

    #     while True:
    #         if self.ser.in_waiting == 0:
    #             continue

    #         line = self.ser.readline().decode(errors="ignore").strip()

    #         # --- update line length stats ---
    #         dbg_total += 1
    #         L = len(line)
    #         dbg_min_len = min(dbg_min_len, L)
    #         dbg_max_len = max(dbg_max_len, L)

    #         is_short = (L < 10)
    #         if is_short:
    #             dbg_short += 1
    #             if len(short_examples) < 5:
    #                 short_examples.append(line)

    #         # 如果很久/很多行都没有短行，给出明确提示（但仍继续等）
    #         if (not warned_no_short) and dbg_short == 0:
    #             if (dbg_total >= warn_after_lines) or ((time.time() - start_time) >= warn_after_seconds):
    #                 self.get_logger().error(
    #                     f"[CALIB-DBG] No short delimiter (len<10) seen after "
    #                     f"{dbg_total} lines / {time.time() - start_time:.1f}s. "
    #                     f"minlen={dbg_min_len}, maxlen={dbg_max_len}. "
    #                     f"This likely means delimiter logic doesn't match device output."
    #                 )
    #                 warned_no_short = True

    #         # --- delimiter handling (original behavior) ---
    #         if is_short:
    #             # 一帧结束：检查 block 是否完整（16行，每行16列）
    #             if len(block) == 16 and all(len(r) == 16 for r in block):
    #                 samples.append(np.array(block))
    #                 calib_blocks.append([row[:] for row in block])

    #                 fps = 1 / (time.time() - t0) if t0 else 0.0
    #                 self.get_logger().debug(f"[RIGHT] fps: {fps:.2f}")
    #                 self.get_logger().info(f"[RIGHT] calib frame collected: {len(samples)}/31")

    #                 if len(samples) >= 31:
    #                     break

    #             # 无论是否完整，都清空 block 等下一帧
    #             block, t0 = [], time.time()
    #             continue

    #         # --- parse numeric row ---
    #         row = self._parse_numeric_row(line)
    #         if row is None:
    #             dbg_nonnum += 1
    #             if len(nonnum_examples) < 5:
    #                 nonnum_examples.append(line)
    #             # 非数字行：建议清空 block 防止错位
    #             block = []
    #             continue

    #         if len(row) != 16:
    #             dbg_otherlen += 1
    #             if len(badlen_examples) < 5:
    #                 badlen_examples.append(line)
    #             # 列数不对：清空 block 防止错位
    #             block = []
    #             continue

    #         dbg_good16 += 1
    #         block.append(row)

    #         # --- periodic report (once per second) ---
    #         now = time.time()
    #         if now - dbg_last_report >= 1.0:
    #             self.get_logger().info(
    #                 f"[CALIB-DBG] total={dbg_total} short(<10)={dbg_short} nonnum={dbg_nonnum} "
    #                 f"good16={dbg_good16} otherlen={dbg_otherlen} minlen={dbg_min_len} maxlen={dbg_max_len} "
    #                 f"block_rows={len(block)} samples={len(samples)}/31"
    #             )

    #             if short_examples:
    #                 self.get_logger().info(f"[CALIB-DBG] short examples: {short_examples}")
    #             if nonnum_examples:
    #                 self.get_logger().info(f"[CALIB-DBG] nonnum examples: {nonnum_examples}")
    #             if badlen_examples:
    #                 self.get_logger().info(f"[CALIB-DBG] badlen examples: {badlen_examples}")

    #             dbg_last_report = now

    #     # --- compute baseline ---
    #     self._median = np.median(np.array(samples), axis=0)

    #     # --- dump calibration frames to JSON ---
    #     dump_obj = {f"{i+1:02d}": calib_blocks[i] for i in range(len(calib_blocks))}
    #     dump_path = Path(self._calib_dump_path)
    #     try:
    #         dump_path.parent.mkdir(parents=True, exist_ok=True)
    #         with dump_path.open("w", encoding="utf-8") as f:
    #             json.dump(dump_obj, f, ensure_ascii=False, indent=2)
    #         self.get_logger().info(f"[RIGHT] calibration frames saved to: {dump_path}")
    #     except Exception as e:
    #         self.get_logger().error(f"[RIGHT] failed to write calib json: {e}")

    #     self.get_logger().info("[RIGHT] calibration complete")


    # def _reader(self) -> None:
    #     samples: List[np.ndarray] = []
    #     block: List[List[int]] = []
    #     calib_blocks: List[List[List[int]]] = []
    #     t0 = 0.0

    #     # debug counters
    #     dbg_total = 0
    #     dbg_short = 0
    #     dbg_nonnum = 0
    #     dbg_good16 = 0
    #     dbg_otherlen = 0
    #     dbg_timeouts = 0
    #     dbg_min_len = 10**9
    #     dbg_max_len = 0
    #     dbg_last_report = time.time()

    #     # --- minimal extra debug state ---
    #     len_hist: dict[int, int] = {}     # 行里数字个数直方图
    #     short_examples: List[str] = []    # 短行样例（最多5条）
    #     self._printed_row32 = getattr(self, "_printed_row32", False)  # 只打印一次32列拆分

    #     self.get_logger().info("[RIGHT] _reader started, entering calibration...")

    #     while True:
    #         raw = self.ser.readline()  # 若你在Serial里设了timeout=1，这里最多等1秒
    #         if raw == b"":
    #             dbg_timeouts += 1
    #             now = time.time()
    #             if now - dbg_last_report >= 1.0:
    #                 self.get_logger().info(
    #                     f"[CALIB-DBG] (no data yet) timeouts={dbg_timeouts} samples={len(samples)}/31 block_rows={len(block)}"
    #                 )
    #                 dbg_last_report = now
    #             continue

    #         line = raw.decode(errors="ignore").strip()

    #         dbg_total += 1
    #         L = len(line)
    #         dbg_min_len = min(dbg_min_len, L)
    #         dbg_max_len = max(dbg_max_len, L)

    #         is_short = (L < 10)
    #         if is_short:
    #             dbg_short += 1
    #             if len(short_examples) < 5:
    #                 short_examples.append(line)

    #         # 每秒固定输出一次统计（不管解析成功与否）
    #         now = time.time()
    #         if now - dbg_last_report >= 1.0:
    #             top_lens = sorted(len_hist.items(), key=lambda kv: kv[1], reverse=True)[:3]
    #             self.get_logger().info(
    #                 f"[CALIB-DBG] total={dbg_total} short(<10)={dbg_short} nonnum={dbg_nonnum} "
    #                 f"good16={dbg_good16} otherlen={dbg_otherlen} timeouts={dbg_timeouts} "
    #                 f"minlen={dbg_min_len} maxlen={dbg_max_len} block_rows={len(block)} samples={len(samples)}/31 "
    #                 f"len_hist_top3={top_lens}"
    #             )
    #             if short_examples:
    #                 self.get_logger().info(f"[CALIB-DBG] short_examples={short_examples}")
    #             dbg_last_report = now

    #         # 短行：按作者原逻辑作为帧分隔符
    #         if is_short:
    #             if len(block) == 16 and all(len(r) == 16 for r in block):
    #                 samples.append(np.array(block))
    #                 calib_blocks.append([row[:] for row in block])

    #                 fps = 1 / (time.time() - t0) if t0 else 0.0
    #                 self.get_logger().debug(f"[RIGHT] fps: {fps:.2f}")
    #                 self.get_logger().info(f"[RIGHT] calib frame collected: {len(samples)}/31")

    #                 if len(samples) >= 31:
    #                     break

    #             block, t0 = [], time.time()
    #             continue

    #         # 数值行解析
    #         row = self._parse_numeric_row(line)
    #         if row is None:
    #             dbg_nonnum += 1
    #             self.get_logger().warn(f"[RIGHT] non-numeric line (calib): {line!r}")
    #             block = []  # 建议清空，避免错位
    #             continue

    #         # 记录行长度分布
    #         len_hist[len(row)] = len_hist.get(len(row), 0) + 1

    #         # 如果看到32列，打印一次拆分结果（帮助判断是不是“两路数据”）
    #         if len(row) == 32 and not self._printed_row32:
    #             self._printed_row32 = True
    #             self.get_logger().info(f"[ROW32] first16={row[:16]}")
    #             self.get_logger().info(f"[ROW32] last16 ={row[16:]}")
    #             self.get_logger().info("[ROW32] Tip: press one spot and see which half changes more.")

    #         # 仍按原逻辑：期望16列，否则当异常（这里只是为了debug确认，不改变主逻辑）
    #         if len(row) != 16:
    #             dbg_otherlen += 1
    #             self.get_logger().warn(f"[RIGHT] bad row length={len(row)} (expect 16): {line!r}")
    #             block = []
    #             continue

    #         dbg_good16 += 1
    #         block.append(row)

    #     # 计算基线 + dump json（保持你原来的逻辑）
    #     self._median = np.median(np.array(samples), axis=0)
    #     dump_obj = {f"{i+1:02d}": calib_blocks[i] for i in range(len(calib_blocks))}
    #     dump_path = Path(self._calib_dump_path)
    #     try:
    #         dump_path.parent.mkdir(parents=True, exist_ok=True)
    #         with dump_path.open("w", encoding="utf-8") as f:
    #             json.dump(dump_obj, f, ensure_ascii=False, indent=2)
    #         self.get_logger().info(f"[RIGHT] calibration frames saved to: {dump_path}")
    #     except Exception as e:
    #         self.get_logger().error(f"[RIGHT] failed to write calib json: {e}")

    #     self.get_logger().info("[RIGHT] calibration complete")

    #     # 下面运行阶段先别动，等你确认分隔符/行格式后再一起改
    #     # ...



    #     # -------- running stage --------
    #     # 先保持你原来的运行逻辑不动（仍用短行分隔），但也要用安全解析避免再炸
    #     block, latest = [], None
    #     while rclpy.ok():
    #         if self.ser.in_waiting == 0:
    #             continue

    #         line = self.ser.readline().decode(errors="ignore").strip()

    #         if len(line) < 10:
    #             if len(block) == 16:
    #                 latest = np.array(block)
    #             block = []

    #             if latest is not None:
    #                 frame = np.clip(latest - self._median - THRESHOLD, 0, 100)
    #                 denom = NOISE_SCALE if np.max(frame) < THRESHOLD else np.max(frame)
    #                 norm = frame / denom

    #                 filt = _temporal_filter(norm, self._prev)
    #                 self._prev = filt

    #                 rotated_filt = np.flipud(np.rot90(filt, k=5))

    #                 msg = TactileInput()
    #                 msg.header.stamp = self.get_clock().now().to_msg()
    #                 msg.local_time = str(time.time())
    #                 msg.data = rotated_filt[:12].flatten().astype(float).tolist()

    #                 self._pub.publish(msg)
    #                 self._last_pub = time.time()

    #             continue

    #         row = self._parse_numeric_row(line)
    #         if row is None:
    #             # 运行阶段非数字行，直接跳过并清空，避免错位
    #             self.get_logger().warn(f"[RIGHT] non-numeric line (run): {line!r}")
    #             block = []
    #             continue
    #         if len(row) != 16:
    #             self.get_logger().warn(f"[RIGHT] bad row length={len(row)} (run): {line!r}")
    #             block = []
    #             continue

    #         block.append(row)

    def _reader(self) -> None:
        samples: List[np.ndarray] = []
        block: List[List[int]] = []
        calib_blocks: List[List[List[int]]] = []
        t0 = 0.0

        # 你现在“原逻辑”期望每行16个数；我们先不改它，只用来判断能否拼帧
        EXPECT_COLS = 16
        EXPECT_ROWS = 16
        TARGET_FRAMES = 31

        # ---- debug counters ----
        total_lines = 0
        short_lines = 0
        nonnum_lines = 0
        numeric_lines = 0
        timeouts = 0
        min_line_len = 10**9
        max_line_len = 0
        last_report = time.time()

        # 数字列数直方图：len(row)->count
        len_hist: dict[int, int] = {}

        # 短行出现时，统计“上一个分隔符到现在累计了多少数值行”
        rows_since_delim = 0
        frame_row_counts: List[int] = []  # 记录最近几次分隔符前累计的行数

        # 只打印少量分析，避免刷屏
        printed_row32 = 0
        printed_short_examples = 0

        self.get_logger().info("[RIGHT] _reader started, entering calibration (debug mode)...")

        # 小工具：相关系数（不依赖numpy也可，但这里用numpy更方便）
        def corr(a: List[int], b: List[int]) -> float:
            aa = np.array(a, dtype=float)
            bb = np.array(b, dtype=float)
            if aa.size == 0 or bb.size == 0:
                return 0.0
            if np.std(aa) < 1e-9 or np.std(bb) < 1e-9:
                return 0.0
            return float(np.corrcoef(aa, bb)[0, 1])

        while True:
            raw = self.ser.readline()  # timeout=1 时，最多等1秒
            if raw == b"":
                timeouts += 1
                now = time.time()
                if now - last_report >= 1.0:
                    top_lens = sorted(len_hist.items(), key=lambda kv: kv[1], reverse=True)[:5]
                    self.get_logger().info(
                        f"[CALIB-DBG] (timeout) timeouts={timeouts} total={total_lines} "
                        f"short={short_lines} nonnum={nonnum_lines} numeric={numeric_lines} "
                        f"line_len[min,max]=[{min_line_len},{max_line_len}] "
                        f"block_rows={len(block)} samples={len(samples)}/{TARGET_FRAMES} "
                        f"len_hist_top={top_lens} rows_since_delim={rows_since_delim} "
                        f"recent_frame_rows={frame_row_counts[-3:] if frame_row_counts else []}"
                    )
                    last_report = now
                continue

            line = raw.decode(errors="ignore").strip()

            total_lines += 1
            L = len(line)
            min_line_len = min(min_line_len, L)
            max_line_len = max(max_line_len, L)

            # --- 短行分隔符 ---
            if L < 10:
                short_lines += 1
                if printed_short_examples < 5:
                    self.get_logger().info(f"[SHORT] example: {line!r}")
                    printed_short_examples += 1

                # 记录这次分隔符前累计的“数值行”数量
                frame_row_counts.append(rows_since_delim)
                rows_since_delim = 0

                # 原逻辑：短行认为一帧结束，检查block是否完整
                if len(block) == EXPECT_ROWS and all(len(r) == EXPECT_COLS for r in block):
                    samples.append(np.array(block))
                    calib_blocks.append([row[:] for row in block])

                    fps = 1 / (time.time() - t0) if t0 else 0.0
                    self.get_logger().info(
                        f"[RIGHT] calib frame collected: {len(samples)}/{TARGET_FRAMES} (fps~{fps:.2f})"
                    )
                    if len(samples) >= TARGET_FRAMES:
                        break

                block, t0 = [], time.time()
                continue

            # --- 数值行解析 ---
            row = self._parse_numeric_row(line)
            if row is None:
                nonnum_lines += 1
                # 这类行会破坏帧对齐，清空block更安全
                self.get_logger().warn(f"[RIGHT] non-numeric line: {line!r}")
                block = []
                rows_since_delim = 0
                continue

            numeric_lines += 1
            rows_since_delim += 1
            len_hist[len(row)] = len_hist.get(len(row), 0) + 1

            # ---- 核心：遇到32列，做“来源判断”分析（只打印前几次）----
            if len(row) == 32 and printed_row32 < 8:
                printed_row32 += 1
                first16 = row[:16]
                last16  = row[16:]
                even16  = row[0::2]  # 0,2,4... 共16个
                odd16   = row[1::2]  # 1,3,5... 共16个

                corr_halves = corr(first16, last16)
                corr_inter  = corr(even16, odd16)

                self.get_logger().info(
                    f"[ROW32#{printed_row32}] halves_corr={corr_halves:.3f} "
                    f"interleave_corr={corr_inter:.3f} "
                    f"mean(first16)={np.mean(first16):.1f} mean(last16)={np.mean(last16):.1f} "
                    f"mean(even16)={np.mean(even16):.1f} mean(odd16)={np.mean(odd16):.1f}"
                )
                self.get_logger().info(f"[ROW32#{printed_row32}] first16={first16}")
                self.get_logger().info(f"[ROW32#{printed_row32}] last16 ={last16}")
                self.get_logger().info(f"[ROW32#{printed_row32}] even16 ={even16}")
                self.get_logger().info(f"[ROW32#{printed_row32}] odd16  ={odd16}")
                self.get_logger().info(
                    "[HINT] 如果 halves_corr 更高，像“前16+后16 两路数据拼接”；"
                    "如果 interleave_corr 更高，像“偶奇交织两路通道”；"
                    "如果分隔符出现时 rows_since_delim 常是 8，可能是 16×16 展平后每行打印32（8行一帧）。"
                )

            # ---- 原逻辑拼帧（不改变你现有规则）----
            if len(row) != EXPECT_COLS:
                # 这里不再每行都warn（太吵），改为只在前几次warn，之后靠len_hist看趋势
                if len_hist[len(row)] <= 5:
                    self.get_logger().warn(f"[RIGHT] row length={len(row)} (expect {EXPECT_COLS}): {line!r}")
                block = []
                rows_since_delim = 0
                continue

            block.append(row)

            # ---- 每秒固定汇总一次（实时反馈）----
            now = time.time()
            if now - last_report >= 1.0:
                top_lens = sorted(len_hist.items(), key=lambda kv: kv[1], reverse=True)[:5]
                self.get_logger().info(
                    f"[CALIB-DBG] total={total_lines} short={short_lines} nonnum={nonnum_lines} numeric={numeric_lines} "
                    f"line_len[min,max]=[{min_line_len},{max_line_len}] "
                    f"block_rows={len(block)} samples={len(samples)}/{TARGET_FRAMES} "
                    f"len_hist_top={top_lens} rows_since_delim={rows_since_delim} "
                    f"recent_frame_rows={frame_row_counts[-3:] if frame_row_counts else []}"
                )
                last_report = now

        # ---- 校准完成后：计算median并dump json ----
        self._median = np.median(np.array(samples), axis=0)
        dump_obj = {f"{i+1:02d}": calib_blocks[i] for i in range(len(calib_blocks))}
        dump_path = Path(self._calib_dump_path)
        try:
            dump_path.parent.mkdir(parents=True, exist_ok=True)
            with dump_path.open("w", encoding="utf-8") as f:
                json.dump(dump_obj, f, ensure_ascii=False, indent=2)
            self.get_logger().info(f"[RIGHT] calibration frames saved to: {dump_path}")
        except Exception as e:
            self.get_logger().error(f"[RIGHT] failed to write calib json: {e}")

        self.get_logger().info("[RIGHT] calibration complete")

        # 后续 running stage 你现在先别接，等你确认“32怎么来的 + 一帧到底几行”以后再改更稳


def main(args=None) -> None:
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads=1)
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
