#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np
from rclpy.duration import Duration

class SafetyMode(Node):
    def __init__(self):
        super().__init__('safety_mode')

        self.laser_sub=self.create_subscription(
LaserScan, '/scan', self.lidar_callback, 10
        )

        self.stop_pub = self.create_publisher(
Twist, '/cmd_vel', 10
        )

    def publish_cmd_vel(self, linear_x, angular_z):
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = float(linear_x)
            cmd_vel_msg.angular.z = float(angular_z)
            
            self.get_logger().info(f"Publishing cmd_vel: linear={linear_x}, angular={angular_z}") 
            self.stop_pub.publish(cmd_vel_msg)

    def lidar_callback(self,msg):
        distances=msg.ranges

        valid_ranges = [
        r for r in distances 
        if msg.range_min <= r <= msg.range_max and not np.isnan(r) and not np.isinf(r)
    ]

        if valid_ranges:
            min_distance = min(valid_ranges)
            self.get_logger().info(f'Najbliższa przeszkoda: {min_distance:.2f} m')

        target_angle = 0.0
        
        if msg.angle_min <= target_angle <= msg.angle_max:
            index = int((target_angle - msg.angle_min) / msg.angle_increment)
            dist_at_angle = distances[index]

            if np.isfinite(dist_at_angle) and msg.range_min <= dist_at_angle <= msg.range_max:
                self.get_logger().info(f'Odległość na wprost (0 rad): {dist_at_angle:.2f} m')
            else:
                self.get_logger().warn('Brak przeszkody wprost przed robotem (poza zakresem / Inf).')

        if dist_at_angle is not None and dist_at_angle<= 0.3:
            self.get_logger().warn('STOP! PRZESZKODA ZA BLISKO')


    

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()