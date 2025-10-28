import json, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('camera_fov_deg', 60.0)
        self.fov = math.radians(self.get_parameter('camera_fov_deg').value)

        self.sub_det = self.create_subscription(String, '/detections', self.cb_dets, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)

        self.pub_target = self.create_publisher(PointStamped, '/fusion/target', 10)

        self.latest_dets = []
        self.latest_scan = None

    def cb_dets(self, msg: String):
        try:
            self.latest_dets = json.loads(msg.data)
        except Exception:
            self.latest_dets = []

        self.compute_target()

    def cb_scan(self, msg: LaserScan):
        self.latest_scan = msg
        self.compute_target()

    def compute_target(self):
        if not self.latest_dets or self.latest_scan is None:
            return

        # pick the largest bbox
        def area(d): 
            x1,y1,x2,y2 = d['bbox']
            return (x2-x1)*(y2-y1)
        det = max(self.latest_dets, key=area)

        # compute angle from bbox center (image width unknown; we infer from bbox range [0..W])
        x1,y1,x2,y2 = det['bbox']
        W = max(x2, x1) * 2  # heuristic; ideally pass image width
        cx = (x1 + x2) / 2.0
        # normalize [-0.5..0.5] across FOV
        x_norm = (cx / W) - 0.25  # crude estimate; better to publish image width
        angle = x_norm * self.fov

        # get lidar distance at this angle
        scan = self.latest_scan
        idx = int((angle - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(idx, len(scan.ranges)-1))
        dist = scan.ranges[idx]

        pt = PointStamped()
        pt.header.frame_id = 'base_link'
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = float(dist) if math.isfinite(dist) else float('nan')
        pt.point.y = float(angle)
        pt.point.z = 0.0
        self.pub_target.publish(pt)

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
