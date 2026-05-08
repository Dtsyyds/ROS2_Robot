#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import message_filters
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class VisionProcessorNode(Node):
    def __init__(self):
        super().__init__('vision_processor_node')
        self.bridge = CvBridge()
        
        # --- 🌟 核心：自检心跳 (兼容模式) ---
        self.camera_status_pub = self.create_publisher(String, 'camera_status', 10)
        
        # 同时订阅原始图和校正图，只要有数据就触发 heartbeat_callback
        # 将 QoS 从 qos_profile_sensor_data 改为 10，确保兼容官方驱动的 Reliable 发布
        self.heartbeat_sub_raw = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.heartbeat_callback, 10)
        
        self.heartbeat_sub_rect = self.create_subscription(
            Image, '/camera/camera/color/image_rect_raw', self.heartbeat_callback, 10)

        # --- 其他原有功能 ---
        self.safety_pub = self.create_publisher(String, '/vision/safety_status', 10)
        self.dist_pub = self.create_publisher(Float32, '/vision/min_dist', 10)
        self.preview_pub = self.create_publisher(Image, '/vision/edge_preview', 10)

        # 图像处理话题：优先尝试 rect_raw (D405)，如果 D435 只有 raw，建议通过重映射统一
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_rect_raw', qos_profile=10)
        
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=10)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)
        
        self.get_logger().info("✅ [相机自检节点] 兼容模式已启动：正在监控 image_raw 与 image_rect_raw...")

    def heartbeat_callback(self, msg):
        """ 只要 image_raw 有数据，就发布在线状态 """
        hb_msg = String()
        hb_msg.data = "online"
        self.camera_status_pub.publish(hb_msg)

    def image_callback(self, color_msg, depth_msg):
        # 保持原有算法逻辑（略）
        pass

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
