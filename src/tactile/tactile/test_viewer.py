#!/usr/bin/env python3
"""Simple OpenCV viewer for left- and right-finger tactile frames."""

from __future__ import annotations

import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rummy_tactile_msgs.msg import TactileInput

SCALE        = 50   # 图像放大倍数
FRAME_SHAPE  = (12, 32)     # 触觉数据的原始形状；在数据发布的时候，裁掉了4行
COLORMAP     = cv2.COLORMAP_VIRIDIS     # 使用Viridis配色方案（蓝-黄渐变）
WIN          = "Tactile Viewer"     # OpenCV窗口名称
STRIPE_W     = 5    # 左右图像间隔条的宽度（像素）
FONT         = cv2.FONT_HERSHEY_SIMPLEX     # 字体类型 
LABEL_POS    = (10, 70)     # 标签位置（x,y）
LABEL_SCALE  = 2    # 字体大小
LABEL_THICK  = 5    # 字体粗细


class TactileViewerNode(Node):
    """Displays left and right tactile inputs side-by-side with a divider."""

    def __init__(self) -> None:
        super().__init__("tactile_viewer_node")

        # self._sub_test = self.create_subscription(  # 创建测试传感器订阅者
        #     TactileInput,"tactile_input_test",self._test_cb,10
        # )

        self._sub_left  = self.create_subscription(     # 创建左侧传感器订阅者
            TactileInput, "tactile_input_left",  self._left_cb, 10
        )
        self._sub_right = self.create_subscription(     # 创建右侧传感器订阅者
            TactileInput, "tactile_input_right", self._right_cb, 10
        )

        self._left_img:  Optional[np.ndarray] = None    # Optional：该变量可以是指定的类型（这里是 np.ndarray），也可以是 None。
        self._right_img: Optional[np.ndarray] = None
        # self._test_img: Optional[np.ndarray] = None

        w, h = FRAME_SHAPE[1] * SCALE, FRAME_SHAPE[0] * SCALE   # 计算窗口尺寸并创建Opencv窗口
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)     # 可调整大小的窗口
        cv2.resizeWindow(WIN, w * 2 + STRIPE_W, h)  # 宽度=左图+右图+间隔条 

    # 回调函数
    def _left_cb(self, msg: TactileInput) -> None: 
        self._left_img = self._to_colormap(msg.data)    # 左触觉数据转彩色图

    def _right_cb(self, msg: TactileInput) -> None:
        self._right_img = self._to_colormap(msg.data)   # 右触觉数据转彩色图

    # def _test_cb(self,msg: TactileInput) -> None:
    #     self._test_img = self._to_colormap(msg.data)   # 测试触觉数据转彩色图

    @staticmethod
    def _to_colormap(flat: list[float]) -> np.ndarray:
        arr = np.array(flat, dtype=np.float32).reshape(FRAME_SHAPE)     # 将扁平化的列表转化成指定形状的Numpy数组
        vis = (arr * 255).astype(np.uint8)      # 将数据归一化到0-255范围并转换为整数类型（图像像素值范围）
        col = cv2.applyColorMap(vis, COLORMAP)      # 应用颜色映射将灰度值转换为彩色图像
        # 调整图像大小并返回
        return cv2.resize(      
            col,
            (FRAME_SHAPE[1] * SCALE, FRAME_SHAPE[0] * SCALE),
            interpolation=cv2.INTER_NEAREST,
        )

    @staticmethod
    def _add_label(img: np.ndarray, text: str) -> np.ndarray:   # 用于在图像上添加文本标签
        out = img.copy()     # 创建图像副本，避免修改原始图像
        cv2.putText(    # 在图像上绘制文本
            out, text.upper(), LABEL_POS, FONT,
            LABEL_SCALE, (255, 255, 255), LABEL_THICK, cv2.LINE_AA
        )
        return out

    def run(self) -> None:
        while rclpy.ok():   # ROS2核心状态检查函数，判断节点是否处于正常运行状态；
            rclpy.spin_once(self, timeout_sec=0.01)     #  ROS 2 节点与主循环的 “接口”，确保节点能正常通信的同时不影响图像刷新。

            if self._left_img is not None and self._right_img is not None :      # 测试图像有效
                # test  = self._add_label(self._test_img,  "test")    # 为图像添加标签（左）
                left  = self._add_label(self._left_img,  "left")    # 为图像添加标签（左）
                right = self._add_label(self._right_img, "right")   # 为图像添加标签（右）
                stripe = np.full((left.shape[0], STRIPE_W, 3), 255, dtype=np.uint8)     # 在左右图像之间添加一条白色分隔线，增强视觉区分度。
                cv2.imshow(WIN, np.hstack((left, stripe,right)))   # 展示图像

            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break

            time.sleep(0.01)

        cv2.destroyAllWindows()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TactileViewerNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
