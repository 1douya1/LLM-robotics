#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def rotation_matrix_to_quaternion(R: np.ndarray):
    q = np.empty(4, dtype=np.float64)
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (R[2, 1] - R[1, 2]) / s
        q[1] = (R[0, 2] - R[2, 0]) / s
        q[2] = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s
    return q[0], q[1], q[2], q[3]


class CharucoPublisher(Node):
    def __init__(self):
        super().__init__('charuco_publisher')
        # Parameters
        self.declare_parameter('squares_x', 5)
        self.declare_parameter('squares_y', 7)
        self.declare_parameter('square_length', 0.025)
        self.declare_parameter('marker_length', 0.018)
        self.declare_parameter('dictionary_name', 'DICT_4X4_50')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('board_frame', 'charuco_board')

        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.board_frame = self.get_parameter('board_frame').get_parameter_value().string_value

        # Topics (use ROS remapping for image/camera_info)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=5)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 'image', self.on_image, qos)
        self.caminfo_sub = self.create_subscription(CameraInfo, 'camera_info', self.on_caminfo, qos)
        self.image_pub = self.create_publisher(Image, 'charuco/result', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # Build Charuco board
        dict_name = self.get_parameter('dictionary_name').get_parameter_value().string_value
        dictionary = self._get_dictionary(dict_name)
        sx = int(self.get_parameter('squares_x').get_parameter_value().integer_value)
        sy = int(self.get_parameter('squares_y').get_parameter_value().integer_value)
        sl = float(self.get_parameter('square_length').get_parameter_value().double_value)
        ml = float(self.get_parameter('marker_length').get_parameter_value().double_value)
        self.board = cv2.aruco.CharucoBoard_create(sx, sy, sl, ml, dictionary)
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.dictionary = dictionary
        self.get_logger().info(f'Charuco board: {sx}x{sy}, square={sl} m, marker={ml} m, dict={dict_name}')

    def _get_dictionary(self, name: str):
        name = name.strip()
        mapping = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
        }
        if name not in mapping:
            self.get_logger().warn(f'Unknown dictionary {name}, fallback to DICT_4X4_50')
            return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        return cv2.aruco.getPredefinedDictionary(mapping[name])

    def on_caminfo(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64).reshape(-1, 1) if msg.d else np.zeros((5, 1), dtype=np.float64)
        self.camera_matrix = K
        self.dist_coeffs = D

    def on_image(self, msg: Image):
        if self.camera_matrix is None:
            return
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_params)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)
            # Refine to ChArUco corners
            retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=self.board
            )
            if retval is not None and charuco_corners is not None and len(charuco_corners) >= 4:
                ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charucoCorners=charuco_corners,
                    charucoIds=charuco_ids,
                    board=self.board,
                    cameraMatrix=self.camera_matrix,
                    distCoeffs=self.dist_coeffs,
                    rvec=None,
                    tvec=None
                )
                if ok:
                    cv2.aruco.drawAxis(cv_img, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                    # Broadcast TF
                    R, _ = cv2.Rodrigues(rvec)
                    x, y, z, w = rotation_matrix_to_quaternion(R)
                    ts = TransformStamped()
                    ts.header.stamp = msg.header.stamp
                    ts.header.frame_id = self.camera_frame
                    ts.child_frame_id = self.board_frame
                    ts.transform.translation.x = float(tvec[0])
                    ts.transform.translation.y = float(tvec[1])
                    ts.transform.translation.z = float(tvec[2])
                    ts.transform.rotation.x = float(x)
                    ts.transform.rotation.y = float(y)
                    ts.transform.rotation.z = float(z)
                    ts.transform.rotation.w = float(w)
                    self.tf_broadcaster.sendTransform(ts)
        # Publish debug/result image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8'))


def main():
    rclpy.init()
    node = CharucoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 