#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft


class RgbdToCupPose(Node):
    def __init__(self):
        super().__init__('rgbd_to_cup_pose')
        # 可调参数
        self.declare_parameter('base_frame', 'link_base')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('u', 640)         # 初始像素列
        self.declare_parameter('v', 360)         # 初始像素行
        self.declare_parameter('roi', 10)        # ROI 半径像素
        self.declare_parameter('max_depth_m', 2.0)
        self.declare_parameter('pub_topic', '/cup_pose')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('orientation_mode', 'base')  # base/camera

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=5)

        # 话题通过 remap 指定
        self.bridge = CvBridge()
        self.sub_depth = self.create_subscription(Image, 'depth', self.on_depth, qos)
        self.sub_caminfo = self.create_subscription(CameraInfo, 'camera_info', self.on_caminfo, qos)

        pub_topic = self.get_parameter('pub_topic').get_parameter_value().string_value
        self.pub_pose = self.create_publisher(PoseStamped, pub_topic, qos)

        self.buf = Buffer()
        self.lst = TransformListener(self.buf, self)

        self.K = None  # fx, fy, cx, cy
        self.depth_img = None
        self.last_stamp = None

        rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / max(1e-3, rate), self.tick)
        self.get_logger().info('rgbd_to_cup_pose started. Remap depth/camera_info accordingly.')

    def on_caminfo(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.K = (K[0, 0], K[1, 1], K[0, 2], K[1, 2])  # fx, fy, cx, cy

    def on_depth(self, msg: Image):
        # 支持 16UC1(mm) 与 32FC1(m)
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if cv_img.dtype == np.uint16:
            self.depth_img = (cv_img.astype(np.float32) / 1000.0)
        else:
            self.depth_img = cv_img.astype(np.float32)
        self.last_stamp = msg.header.stamp

    def backproject(self, u: int, v: int, Z: float):
        fx, fy, cx, cy = self.K
        X = (u - cx) / fx * Z
        Y = (v - cy) / fy * Z
        return X, Y, Z

    def tick(self):
        if self.K is None or self.depth_img is None:
            return
        h, w = self.depth_img.shape[:2]
        u = int(self.get_parameter('u').get_parameter_value().integer_value)
        v = int(self.get_parameter('v').get_parameter_value().integer_value)
        r = int(self.get_parameter('roi').get_parameter_value().integer_value)
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(f'u,v 越界: ({u},{v}) 超出图像大小 ({w}x{h})')
            return
        u0, v0 = max(0, u - r), max(0, v - r)
        u1, v1 = min(w, u + r + 1), min(h, v + r + 1)
        roi = self.depth_img[v0:v1, u0:u1]
        if roi.size == 0:
            return
        # 过滤无效深度
        max_d = float(self.get_parameter('max_depth_m').get_parameter_value().double_value)
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > 0.05) & (valid < max_d)]
        if valid.size < 10:
            # 可能 ROI 在空白区域
            return
        Z = float(np.median(valid))
        X, Y, Z = self.backproject(u, v, Z)

        camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # 查表: base <- camera 的 TF
        try:
            tf = self.buf.lookup_transform(
                base_frame, camera_frame, Time(), timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f'lookup_transform 失败: {e}')
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        Tbc = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        Tbc[0, 3] = t.x
        Tbc[1, 3] = t.y
        Tbc[2, 3] = t.z
        p_cam = np.array([X, Y, Z, 1.0], dtype=np.float64)
        p_base = Tbc @ p_cam

        ps = PoseStamped()
        ps.header.frame_id = base_frame
        ps.header.stamp = self.last_stamp if self.last_stamp is not None else self.get_clock().now().to_msg()
        ps.pose.position.x = float(p_base[0])
        ps.pose.position.y = float(p_base[1])
        ps.pose.position.z = float(p_base[2])

        mode = self.get_parameter('orientation_mode').get_parameter_value().string_value
        if mode == 'camera':
            # 简化：朝向保持为基坐标对齐
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        self.pub_pose.publish(ps)


def main():
    rclpy.init()
    n = RgbdToCupPose()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 