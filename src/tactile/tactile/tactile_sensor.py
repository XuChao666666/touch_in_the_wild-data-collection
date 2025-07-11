#!/usr/bin/env python3
"""ROS 2 publishers for left and right tactile sensors."""

import time
import threading
from typing import List

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
    """Simple exponential smoothing."""
    return alpha * new + (1 - alpha) * prev


class TactileSensorNodeLeft(Node):
    def __init__(self) -> None:
        super().__init__("tactile_sensor_node_left")
        self._pub = self.create_publisher(TactileInput, "tactile_input_left", 10)

        self.declare_parameter("tactile_sensor_name_left", "left_finger_real")
        dev = self.get_parameter("tactile_sensor_name_left").value
        self.get_logger().info(f"[LEFT] /dev/{dev}")

        try:
            self.ser = serial.Serial(f"/dev/{dev}", BAUD)
        except serial.SerialException as err:
            self.get_logger().error(f"[LEFT] {err}")
            raise

        self._prev = np.zeros((16, 32))
        self._median: np.ndarray | None = None
        self._last_pub = time.time()

        threading.Thread(target=self._reader, daemon=True).start()

    @staticmethod
    def _blur(arr: np.ndarray, sigma: float = 0.1) -> np.ndarray:  
        return gaussian_filter(arr, sigma=sigma)

    def _reader(self) -> None:
        samples: List[np.ndarray] = []
        block:   List[List[int]]  = []
        t0 = 0.0

        while True:
            if self.ser.in_waiting == 0:
                continue
            line = self.ser.readline().decode(errors="ignore").strip()
            if len(line) < 10:
                if len(block) == 16 and all(len(r) == 32 for r in block):
                    samples.append(np.array(block))
                    fps = 1 / (time.time() - t0) if t0 else 0.0
                    self.get_logger().debug(f"[LEFT] fps: {fps:.2f}")
                    if len(samples) > 30:
                        break
                block, t0 = [], time.time()
                continue
            block.append([int(v) for v in line.split()])

        self._median = np.median(np.array(samples), axis=0)
        self.get_logger().info("[LEFT] calibration complete")

        block, latest = [], None
        while rclpy.ok():
            if self.ser.in_waiting == 0:
                continue
            line = self.ser.readline().decode(errors="ignore").strip()
            if len(line) < 10:
                if len(block) == 16:
                    latest = np.array(block)
                block = []
                if latest is not None:
                    frame = np.clip(latest - self._median - THRESHOLD, 0, 100)
                    norm  = frame / (NOISE_SCALE if np.max(frame) < THRESHOLD else np.max(frame))
                    filt  = _temporal_filter(norm, self._prev); self._prev = filt

                    msg = TactileInput()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.local_time   = str(time.time())
                    msg.data = filt[4:].flatten().astype(float).tolist()
                    self._pub.publish(msg)
                    self._last_pub = time.time()
                continue
            block.append([int(v) for v in line.split()])


class TactileSensorNodeRight(Node):
    def __init__(self) -> None:
        super().__init__("tactile_sensor_node_right")
        self._pub = self.create_publisher(TactileInput, "tactile_input_right", 10)

        self.declare_parameter("tactile_sensor_name_right", "right_finger")
        dev = self.get_parameter("tactile_sensor_name_right").value
        self.get_logger().info(f"[RIGHT] /dev/{dev}")

        try:
            self.ser = serial.Serial(f"/dev/{dev}", BAUD)
        except serial.SerialException as err:
            self.get_logger().error(f"[RIGHT] {err}")
            raise

        self._prev = np.zeros((16, 32))
        self._median: np.ndarray | None = None
        self._last_pub = time.time()

        threading.Thread(target=self._reader, daemon=True).start()

    @staticmethod
    def _blur(arr: np.ndarray, sigma: float = 0.1) -> np.ndarray:
        return gaussian_filter(arr, sigma=sigma)

    def _reader(self) -> None:
        samples: List[np.ndarray] = []
        block:   List[List[int]]  = []
        t0 = 0.0

        while True:
            if self.ser.in_waiting == 0:
                continue
            line = self.ser.readline().decode(errors="ignore").strip()
            if len(line) < 10:
                if len(block) == 16 and all(len(r) == 32 for r in block):
                    samples.append(np.array(block))
                    fps = 1 / (time.time() - t0) if t0 else 0.0
                    self.get_logger().debug(f"[RIGHT] fps: {fps:.2f}")
                    if len(samples) > 30:
                        break
                block, t0 = [], time.time()
                continue
            block.append([int(v) for v in line.split()])

        self._median = np.median(np.array(samples), axis=0)
        self.get_logger().info("[RIGHT] calibration complete")

        block, latest = [], None
        while rclpy.ok():
            if self.ser.in_waiting == 0:
                continue
            line = self.ser.readline().decode(errors="ignore").strip()
            if len(line) < 10:
                if len(block) == 16:
                    latest = np.array(block)
                block = []
                if latest is not None:
                    frame = np.clip(latest - self._median - THRESHOLD, 0, 100)
                    norm  = frame / (NOISE_SCALE if np.max(frame) < THRESHOLD else np.max(frame))
                    filt  = _temporal_filter(norm, self._prev); self._prev = filt

                    msg = TactileInput()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.local_time   = str(time.time())
                    msg.data = filt[4:].flatten().astype(float).tolist()
                    self._pub.publish(msg)
                    self._last_pub = time.time()
                continue
            block.append([int(v) for v in line.split()])


def main(args=None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
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
