import json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np

from ..utils.fake_detector import FakeDetector
from ..utils.yolov8_detector import YOLODetector

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.declare_parameters(namespace='', parameters=[
            ('backend', 'auto'),
            ('model_path', 'models/best.pt'),
            ('conf_thres', 0.25),
            ('iou_thres', 0.45),
        ])
        backend = self.get_parameter('backend').get_parameter_value().string_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        conf = self.get_parameter('conf_thres').get_parameter_value().double_value
        iou = self.get_parameter('iou_thres').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.detector = None
        if backend == 'yolo':
            self.detector = YOLODetector(model_path, conf=conf, iou=iou)
        elif backend == 'fake':
            self.detector = FakeDetector()
        else:
            # auto
            try:
                self.detector = YOLODetector(model_path, conf=conf, iou=iou)
                self.get_logger().info('Using YOLO backend')
            except Exception as e:
                self.get_logger().warn(f'YOLO unavailable ({e}); using FakeDetector')
                self.detector = FakeDetector()

        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.cb_image, 10)
        self.pub_det = self.create_publisher(String, '/detections', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/detections_markers', 10)

    def cb_image(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        dets = self.detector.infer(cv_img)
        # Publish JSON list
        s = String()
        s.data = json.dumps(dets)
        self.pub_det.publish(s)

        # Markers for RViz
        h, w = cv_img.shape[:2]
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, d in enumerate(dets):
            x1,y1,x2,y2 = d['bbox']
            m = Marker()
            m.header.frame_id = 'camera_frame'
            m.header.stamp = now
            m.ns = 'detections'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            # project to a fixed plane in front of camera for visualization
            z = 0.5
            m.pose.position.x = z
            m.pose.position.y = ((x1+x2)/2 - w/2) / (w/2) * 0.5
            m.pose.position.z = ((y1+y2)/2 - h/2) / (h/2) * 0.5
            m.scale.x = 0.05; m.scale.y = 0.05; m.scale.z = 0.05
            m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8
            markers.markers.append(m)
        self.pub_markers.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
