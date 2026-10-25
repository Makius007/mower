#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
import tf2_ros
import numpy as np
from rclpy.duration import Duration
from mower_interfaces.msg import TagMeasurement
import math

class TagLocalization(Node):
    def __init__(self):
        super().__init__('tag_localization')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.apriltag_sub = self.create_subscription(AprilTagDetectionArray, '/detections', self.apriltag_callback, 10)
        self.tag_measure_pub = self.create_publisher(TagMeasurement, '/tag_measurement', 10)


    def apriltag_callback(self, msg):
        if not msg.detections:
            return
        stamp = msg.header.stamp
        for detection in msg.detections:
            tag_id = detection.id
            tag_frame = f'tag36h11_{tag_id}'

            try:
                tf_tag_base = self.tf_buffer.lookup_transform(
                    tag_frame, 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.1)
                )

                p_tb = tf_tag_base.transform.translation
                q_tb = tf_tag_base.transform.rotation
            except Exception as e:
                self.get_logger().debug(f"Brak TF: {tag_frame} -> base_link | Powód: {e}")
                continue

            
            
            dx = p_tb.x
            dy = p_tb.y
            dz = p_tb.z
            r = math.sqrt(dx*dx + dy*dy + dz*dz)

            fx = 1.0 - 2.0 * (q_tb.y*q_tb.y + q_tb.z*q_tb.z)
            fz = 2.0 * (q_tb.x*q_tb.z - q_tb.w*q_tb.y)
            yaw = math.atan2(fx, -fz)

            out = TagMeasurement()
            out.header.stamp = stamp
            out.header.frame_id = 'base_link'
            out.id = tag_id
            out.range = r
            out.distance_to_wall = dz
            out.offset_along_wall = dx
            out.heading_error = yaw
            self.tag_measure_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = TagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


