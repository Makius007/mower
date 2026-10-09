#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class GrassBoundaryDetector(Node):
    def __init__(self):
        super().__init__("grass_boundary_detector")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("h_min", 35)
        self.declare_parameter("s_min", 40)
        self.declare_parameter("v_min", 35)
        self.declare_parameter("h_max", 95)
        self.declare_parameter("s_max", 255)
        self.declare_parameter("v_max", 255)
        self.declare_parameter("roi_start_fraction", 0.50)
        self.declare_parameter("kernel_size", 5)

        image_topic = self.get_parameter("image_topic").value

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )

        self.mask_pub = self.create_publisher(
            Image,
            "/mower/grass_mask",
            10,
        )

        self.debug_pub = self.create_publisher(
            Image,
            "/mower/grass_debug",
            10,
        )

        self.get_logger().info(
            f"Grass detector listening on: {image_topic}"
        )

    def image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        height, width = bgr.shape[:2]

        roi_start_fraction = float(
            self.get_parameter("roi_start_fraction").value
        )
        roi_start = int(height * roi_start_fraction)

        roi = bgr[roi_start:height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_green = np.array([
            int(self.get_parameter("h_min").value),
            int(self.get_parameter("s_min").value),
            int(self.get_parameter("v_min").value),
        ])

        upper_green = np.array([
            int(self.get_parameter("h_max").value),
            int(self.get_parameter("s_max").value),
            int(self.get_parameter("v_max").value),
        ])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        kernel_size = int(self.get_parameter("kernel_size").value)
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        debug = bgr.copy()

        roi_debug = debug[roi_start:height, :]

        overlay = roi_debug.copy()
        overlay[mask > 0] = (0, 255, 0)

        debug[roi_start:height, :] = cv2.addWeighted(
            roi_debug,
            0.55,
            overlay,
            0.45,
            0.0,
        )

        cv2.rectangle(
            debug,
            (0, roi_start),
            (width - 1, height - 1),
            (255, 0, 0),
            2,
        )

        full_mask = np.zeros((height, width), dtype=np.uint8)
        full_mask[roi_start:height, :] = mask

        try:
            mask_msg = self.bridge.cv2_to_imgmsg(
                full_mask,
                encoding="mono8",
            )
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

            debug_msg = self.bridge.cv2_to_imgmsg(
                debug,
                encoding="bgr8",
            )
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

        except Exception as exc:
            self.get_logger().error(f"Image publish failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = GrassBoundaryDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()