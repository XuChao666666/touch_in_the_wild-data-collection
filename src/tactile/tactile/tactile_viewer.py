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

SCALE        = 50
FRAME_SHAPE  = (12, 32)  
COLORMAP     = cv2.COLORMAP_VIRIDIS
WIN          = "Tactile Viewer"
STRIPE_W     = 5              
FONT         = cv2.FONT_HERSHEY_SIMPLEX
LABEL_POS    = (10, 70)   
LABEL_SCALE  = 2
LABEL_THICK  = 5


class TactileViewerNode(Node):
    """Displays left and right tactile inputs side-by-side with a divider."""

    def __init__(self) -> None:
        super().__init__("tactile_viewer_node")

        self._sub_left  = self.create_subscription(
            TactileInput, "tactile_input_left",  self._left_cb, 10
        )
        self._sub_right = self.create_subscription(
            TactileInput, "tactile_input_right", self._right_cb, 10
        )

        self._left_img:  Optional[np.ndarray] = None
        self._right_img: Optional[np.ndarray] = None

        w, h = FRAME_SHAPE[1] * SCALE, FRAME_SHAPE[0] * SCALE
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN, w * 2 + STRIPE_W, h)

    def _left_cb(self, msg: TactileInput) -> None:
        self._left_img = self._to_colormap(msg.data)

    def _right_cb(self, msg: TactileInput) -> None:
        self._right_img = self._to_colormap(msg.data)

    @staticmethod
    def _to_colormap(flat: list[float]) -> np.ndarray:
        arr = np.array(flat, dtype=np.float32).reshape(FRAME_SHAPE)
        vis = (arr * 255).astype(np.uint8)
        col = cv2.applyColorMap(vis, COLORMAP)
        return cv2.resize(
            col,
            (FRAME_SHAPE[1] * SCALE, FRAME_SHAPE[0] * SCALE),
            interpolation=cv2.INTER_NEAREST,
        )

    @staticmethod
    def _add_label(img: np.ndarray, text: str) -> np.ndarray:
        out = img.copy()
        cv2.putText(
            out, text.upper(), LABEL_POS, FONT,
            LABEL_SCALE, (255, 255, 255), LABEL_THICK, cv2.LINE_AA
        )
        return out

    def run(self) -> None:
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self._left_img is not None and self._right_img is not None:
                left  = self._add_label(self._left_img,  "left")
                right = self._add_label(self._right_img, "right")
                stripe = np.full((left.shape[0], STRIPE_W, 3), 255, dtype=np.uint8)
                cv2.imshow(WIN, np.hstack((left, stripe, right)))

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
