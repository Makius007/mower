#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from tf2_ros import TransformBroadcaster

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.declare_parameter('wheel_radius',0.1)
        self.declare_parameter('wheel_seperation', 0.449)
        self.declare_parameter('left_joint', 'left_wheel_joint')
        self.declare_parameter('right_joint', 'right_wheel_joint')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_seperation = self.get_parameter('wheel_seperation').value
        self.left_joint = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value

        self.x=0.0
        self.y=0.0
        self.theta=0.0
        self.v=0.0
        self.w=0.0
        self.prev_pos_l = 0.0
        self.prev_pos_p = 0.0

        self.timeout = 0.5
        self.last_cmd_time = self.get_clock().now()
        self.prev_t = None

        self.vel_pub = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

        self.timer = self.create_timer(0.02, self.command_timer_callback)
        

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.get_logger().debug(f"Predkosc liniowa v: {msg.linear.x}, predkosc katowa w: {msg.angular.z}")

    def command_timer_callback(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.timeout:
            v = 0.0
            w = 0.0
        else:
            v = self.v
            w = self.w

        wl = (v - w * self.wheel_seperation / 2.0) / self.wheel_radius
        wp = (v + w * self.wheel_seperation / 2.0) / self.wheel_radius

        msg = Float64MultiArray()
        msg.data = [wl, wp]
        self.vel_pub.publish(msg)

    def joint_states_callback(self, msg):
        try:
            il = msg.name.index(self.left_joint)
            ip = msg.name.index(self.right_joint)
        except ValueError:
            return
        if not msg.position:
            return
        
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos_l = msg.position[il]
        pos_p = msg.position[ip]

        if self.prev_t is None or t < self.prev_t:
            self.prev_t = t
            self.prev_pos_l = pos_l
            self.prev_pos_p = pos_p
            return

        dt = t - self.prev_t

        if dt <= 0.0:
            return
        
        dl = pos_l - self.prev_pos_l
        dp = pos_p - self.prev_pos_p

        self.prev_t = t
        self.prev_pos_l = pos_l
        self.prev_pos_p = pos_p

        ds = self.wheel_radius*(dp+dl)/2
        d_theta = self.wheel_radius*(dp-dl)/(self.wheel_seperation)

        self.x += ds * math.cos(self.theta + d_theta / 2.0)
        self.y += ds * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        v = ds / dt
        w = d_theta / dt

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id="odom"
        odom.child_frame_id="base_link" 
        odom.pose.pose.position.x=self.x
        odom.pose.pose.position.y=self.y
        odom.pose.pose.position.z=0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        t_msg = TransformStamped()
        t_msg.header.stamp = msg.header.stamp
        t_msg.header.frame_id = 'odom'
        t_msg.child_frame_id = 'base_link'
        t_msg.transform.translation.x = self.x
        t_msg.transform.translation.y = self.y
        t_msg.transform.rotation.z = math.sin(self.theta / 2.0)
        t_msg.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t_msg)

def main():
    rclpy.init()
    node = DriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()