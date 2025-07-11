#!/usr/bin/env python3
"""
QR Code Generator Node for ROS2

This node generates a QR code containing the current ROS2 time (formatted in ISO 8601)
and displays it using OpenCV. Update rate is set to 10 Hz (every 100 ms).
Press 'q' in the QR code window to shut down the node.
"""

import cv2
import numpy as np
import qrcode
import datetime

import rclpy
from rclpy.node import Node

class QRCodeGeneratorNode(Node):
    def __init__(self):
        super().__init__('qr_code_generator')
        self.get_logger().info("QR Code Generator Node started. Displaying updated QR code.")
        # Set the update period (in seconds); here 0.1 sec = 10 Hz update rate.
        self.timer_period = 1/30
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # Get the current ROS2 time.
        now_ros = self.get_clock().now()
        # For display purposes, use system time in UTC.
        now_utc = datetime.datetime.now(datetime.timezone.utc)
        timestamp_str = now_utc.strftime('%Y-%m-%dT%H:%M:%S.%fZ')
        self.get_logger().debug(f"Current timestamp: {timestamp_str}")

        # Generate a QR code from the timestamp string.
        # Increase box_size to make the QR code larger.
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_L,
            box_size=40,  # Larger box size makes the code display bigger
            border=4
        )
        qr.add_data(timestamp_str)
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white")

        # Convert the PIL image to a NumPy array (in RGB format).
        qr_image = np.array(img.convert('RGB'))

        # (Optional) Additional scaling with OpenCV if you want an even bigger image:
        # scale_factor = 2
        # height, width, _ = qr_image.shape
        # qr_image = cv2.resize(qr_image, (width * scale_factor, height * scale_factor), interpolation=cv2.INTER_NEAREST)

        # Display the QR code using OpenCV.
        cv2.imshow("QR Code", qr_image)
        # cv2.waitKey() with a short delay is needed to refresh the window.
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.get_logger().info("Received 'q' keypress. Shutting down QR code generator node.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Exiting node...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
