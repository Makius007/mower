#!/usr/bin/env python3
import math
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class MowingAlgorithm(Node):

    STATE_FORWARD = "FORWARD"
    STATE_TURN_OUT = "TURN_OUT"
    STATE_SHIFT_LANE = "SHIFT_LANE"
    STATE_TURN_IN = "TURN_IN"

    def __init__(self):
        super().__init__('mowing_algorithm')

        # Parametry
        self.declare_parameter("grass_threshold", 0.3)
        self.declare_parameter("mower_width", 0.449)

        # Pamięć stanów i odometrii
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.start_x = 0.0
        self.start_y = 0.0

        self.target_yaw = 0.0
        self.is_green_ahead = True
        self.first_run = True
        self.right = True
        self.state = self.STATE_FORWARD

        self.bridge = CvBridge()

        # Subskrypcje
        self.cam_sub = self.create_subscription(
            Image,
            '/mower/grass_mask',
            self.mask_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Publikacja
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Pętla sterowania 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def turn_offset(self, angle):
        return -angle if self.right else angle

    def turn_to(self, target_yaw, cmd):
        diff = self.normalize_angle(target_yaw - self.current_yaw)
        error = abs(diff)
        if error <= 0.03:
            cmd.angular.z = 0.0
            return True
        Kp = 1.2
        ang_vel = max(0.35, min(1.0, Kp * error))
        cmd.angular.z = ang_vel if diff > 0 else -ang_vel
        return False

    def mask_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        height, _ = cv_image.shape
        roi_start = int(height * 0.7)
        bottom_region = cv_image[roi_start:height, :]

        white_pixels = np.count_nonzero(bottom_region)
        total_pixels = bottom_region.size
        grass_ratio = white_pixels / total_pixels if total_pixels > 0 else 0

        grass_threshold = self.get_parameter("grass_threshold").value

        # Aktualizacja flagi na podstawie tego, co FAKTYCZNIE jest przed robotem
        self.is_green_ahead = grass_ratio >= grass_threshold

    def odom_callback(self, msg: Odometry):
        # Pobranie pozycji
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Pobranie kwaternionu z fuzji i konwersja na Yaw
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def control_loop(self):
        cmd = Twist()

        # ---------------------------------------------------------
        # STAN 1: KOSZENIE PROSTO
        # ---------------------------------------------------------
        if self.state == self.STATE_FORWARD:
            if self.is_green_ahead:
                cmd.linear.x = 0.5

                if self.first_run:
                    self.target_yaw = self.current_yaw
                    self.first_run = False
                    self.get_logger().info(f"Zablokowano początkowy kurs: {math.degrees(self.target_yaw):.1f} stopni")

                # Stabilizacja toru jazdy na wprost (Heading Lock)
                heading_error = self.normalize_angle(self.target_yaw - self.current_yaw)
                cmd.angular.z = 0.8 * heading_error

            else:
                self.get_logger().info("Brak trawy - przerywam koszenie, rozpoczynam nawrót (TURN_OUT).")
                self.state = self.STATE_TURN_OUT

        # ---------------------------------------------------------
        # STAN 2: PIERWSZY OBRÓT O 90 STOPNI
        # ---------------------------------------------------------
        elif self.state == self.STATE_TURN_OUT:
            turn_yaw = self.normalize_angle(self.target_yaw + self.turn_offset(math.pi / 2.0))

            if self.turn_to(turn_yaw, cmd):
                self.get_logger().info("Obrót 1 zakończony.")
                # Zapisanie pozycji startowej do przejazdu na nowy pas
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.state = self.STATE_SHIFT_LANE

        # ---------------------------------------------------------
        # STAN 3: PRZEJAZD NA NOWY PAS
        # ---------------------------------------------------------
        elif self.state == self.STATE_SHIFT_LANE:
            shift = math.hypot(self.current_x - self.start_x, self.current_y - self.start_y)
            shift_yaw = self.normalize_angle(self.target_yaw + self.turn_offset(math.pi / 2.0))
            target_shift = self.get_parameter("mower_width").value / 2.0

            if shift < target_shift:
                cmd.linear.x = 0.35
                # Jazda po prostej linii w trakcie przesunięcia
                heading_error = self.normalize_angle(shift_yaw - self.current_yaw)
                cmd.angular.z = 0.8 * heading_error
            else:
                self.get_logger().info("Przesunięcie zakończone, rozpoczynam wejście w pas (TURN_IN).")
                self.state = self.STATE_TURN_IN

        # ---------------------------------------------------------
        # STAN 4: DRUGI OBRÓT O 90 STOPNI (ŁĄCZNIE 180)
        # ---------------------------------------------------------
        elif self.state == self.STATE_TURN_IN:
            turn_yaw = self.normalize_angle(self.target_yaw + self.turn_offset(math.pi))

            if self.turn_to(turn_yaw, cmd):
                self.get_logger().info("Nawrót zakończony, powrót do koszenia przed siebie.")
                # Uaktualnienie globalnego wektora i zmiana kierunku na następny nawrót
                self.target_yaw = turn_yaw
                self.right = not self.right
                self.state = self.STATE_FORWARD

        self.vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MowingAlgorithm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()