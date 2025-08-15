#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
import sys
import termios
import tty

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.publisher = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        self.speed = 3  # m/s eşdeğeri, seninkine göre ayarla
        self.get_logger().info("WASD ile kontrol başlatıldı. 'q' ile çık.")

        # Dosya yolunu belirt
        self.lidar_file = open('/home/halit/acrome_ws/src/acrome_mini_robot/launch/lidar_data.txt', 'a')

    def publish(self, left, right):
        msg = Float64MultiArray()
        msg.data = [left, right]
        self.publisher.publish(msg)
        self.get_logger().info(f"Komut gönderildi: {msg.data}")

    def lidar_callback(self, msg: LaserScan):
        ranges_sample = msg.ranges[:10]
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        line = f"{timestamp}," + ",".join(f"{r:.3f}" for r in ranges_sample) + "\n"
        self.lidar_file.write(line)
        self.lidar_file.flush()
        self.get_logger().info(f"Lidar verisi dosyaya yazıldı: {ranges_sample}")

    def destroy_node(self):
        self.lidar_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        while rclpy.ok():
            key = get_key()
            if key.lower() == 'w':  # ileri
                node.publish(-node.speed, -node.speed)
            elif key.lower() == 's':  # geri
                node.publish(node.speed, node.speed)
            elif key.lower() == 'a':  # sola
                node.publish(node.speed, -node.speed)
            elif key.lower() == 'd':  # sağa
                node.publish(-node.speed, node.speed)
            elif key.lower() == 'q':  # çıkış
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
