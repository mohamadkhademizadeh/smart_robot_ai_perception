import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.last_scan = None

    def cb_scan(self, msg: LaserScan):
        # cache the scan for fusion; for simplicity we keep it internal
        self.last_scan = msg

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
