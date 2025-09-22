import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import socket, json, zlib, base64, uuid

UDP_IP = "172.20.10.3"   # Raspberry IP
UDP_PORT = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

CHUNK_SIZE = 1200  # güvenli UDP fragment boyutu

def compress_and_split(data_dict, topic_name):
    compressed = base64.b64encode(zlib.compress(json.dumps(data_dict).encode()))
    msg_id = str(uuid.uuid4())
    total_chunks = (len(compressed) // CHUNK_SIZE) + 1
    packets = []
    for i in range(total_chunks):
        chunk = compressed[i*CHUNK_SIZE:(i+1)*CHUNK_SIZE]
        packets.append(json.dumps({
            "topic": topic_name,
            "id": msg_id,
            "seq": i,
            "total": total_chunks,
            "data": chunk.decode()
        }).encode())
    return packets

class DataSender(Node):
    def __init__(self):
        super().__init__('data_sender')
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def scan_callback(self, msg):
        data = {
            "header": {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id,
            },
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "time_increment": msg.time_increment,
            "scan_time": msg.scan_time,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "ranges": list(msg.ranges),
            "intensities": list(msg.intensities),
        }
        for packet in compress_and_split(data, "scan"):
            sock.sendto(packet, (UDP_IP, UDP_PORT))

    def odom_callback(self, msg):
        data = {
            "header": {
                "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                "frame_id": msg.header.frame_id,
            },
            "child_frame_id": msg.child_frame_id,
            "pose": {
                "pose": {
                    "position": {
                        "x": msg.pose.pose.position.x,
                        "y": msg.pose.pose.position.y,
                        "z": msg.pose.pose.position.z,
                    },
                    "orientation": {
                        "x": msg.pose.pose.orientation.x,
                        "y": msg.pose.pose.orientation.y,
                        "z": msg.pose.pose.orientation.z,
                        "w": msg.pose.pose.orientation.w,
                    }
                }
            },
            "twist": {
                "twist": {
                    "linear": {
                        "x": msg.twist.twist.linear.x,
                        "y": msg.twist.twist.linear.y,
                        "z": msg.twist.twist.linear.z,
                    },
                    "angular": {
                        "x": msg.twist.twist.angular.x,
                        "y": msg.twist.twist.angular.y,
                        "z": msg.twist.twist.angular.z,
                    }
                }
            }
        }
        for packet in compress_and_split(data, "odom"):
            sock.sendto(packet, (UDP_IP, UDP_PORT))

def main():
    rclpy.init()
    node = DataSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()