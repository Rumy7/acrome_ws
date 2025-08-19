#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import os
import csv

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Parametreler
        self.wheel_radius = 0.04      # URDF'den alındı
        self.wheel_separation = 0.25  # Tekerlekler arası mesafe
        self.base_frame = "base_link"
        self.odom_frame = "odom"

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Joint state subscribe
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_left_pos = None
        self.last_right_pos = None

        # --- Dosya kaydı ---
        self.logfile = os.path.expanduser(
            "/home/halit/acrome_ws/src/acrome_mini_robot/nodes/nodesodom_log.csv"
        )
        self.file = open(self.logfile, "w", newline="")
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(["time_sec", "x", "y", "theta", "v", "w"])  # başlık

    def joint_state_callback(self, msg):
        # wheel_2 = left, wheel_3 = right
        try:
            left_idx = msg.name.index("base_to_wheel_2")
            right_idx = msg.name.index("base_to_wheel_3")
        except ValueError:
            self.get_logger().warning("Joint names not found in message")
            return

        left_pos = msg.position[left_idx]
        right_pos = msg.position[right_idx]

        if self.last_left_pos is None:
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            return

        # Diferansiyel sürüş kinematiği
        dl = (left_pos - self.last_left_pos) * self.wheel_radius
        dr = (right_pos - self.last_right_pos) * self.wheel_radius
        self.last_left_pos = left_pos
        self.last_right_pos = right_pos

        dc = (dr + dl) / 2.0
        dtheta = (dr - dl) / self.wheel_separation

        # Pose güncelle
        self.x += dc * math.cos(self.theta + dtheta / 2.0)
        self.y += dc * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        # Zaman
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Twist
        vx = dc / dt if dt > 0 else 0.0
        vth = dtheta / dt if dt > 0 else 0.0

        # Odometry mesajı
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        # --- Dosyaya yaz ---
        time_sec = current_time.seconds_nanoseconds()[0] + current_time.seconds_nanoseconds()[1]*1e-9
        self.csv_writer.writerow([time_sec, self.x, self.y, self.theta, vx, vth])
        self.file.flush()  # anında diske yaz

        # TF publish
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
