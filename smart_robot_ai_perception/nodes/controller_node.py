import time, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from ..utils.pid import PID  # relative import

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameters(namespace='', parameters=[
            ('kp_ang', 1.2), ('ki_ang', 0.0), ('kd_ang', 0.05),
            ('kp_lin', 0.8), ('ki_lin', 0.0), ('kd_lin', 0.02),
            ('max_lin', 0.4), ('max_ang', 1.5),
            ('target_stop_range', 0.6),
        ])
        self.pid_ang = PID(self.get_parameter('kp_ang').value,
                           self.get_parameter('ki_ang').value,
                           self.get_parameter('kd_ang').value)
        self.pid_lin = PID(self.get_parameter('kp_lin').value,
                           self.get_parameter('ki_lin').value,
                           self.get_parameter('kd_lin').value)
        self.max_lin = self.get_parameter('max_lin').value
        self.max_ang = self.get_parameter('max_ang').value
        self.stop_range = self.get_parameter('target_stop_range').value

        self.sub_target = self.create_subscription(PointStamped, '/fusion/target', self.cb_target, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_time = self.get_clock().now()

    def cb_target(self, msg: PointStamped):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        angle_err = msg.point.y  # radians
        dist = msg.point.x if math.isfinite(msg.point.x) else float('inf')

        # Angular control: turn to face target
        w = self.pid_ang.step(angle_err, dt)
        w = max(min(w, self.max_ang), -self.max_ang)

        # Linear control: move forward until stop range
        v_err = dist - self.stop_range
        v = self.pid_lin.step(v_err, dt)
        v = max(min(v, self.max_lin), 0.0)  # don't reverse in this simple demo

        if not math.isfinite(dist) or dist < self.stop_range:
            v = 0.0

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
