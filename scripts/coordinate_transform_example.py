#!/usr/bin/env python3
"""
物体坐标转换示例.

将 find_object_2d 检测到的物体坐标从相机坐标系转换到机器人基坐标系
"""

import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from find_object_2d.msg import ObjectsStamped, DetectionInfo
import numpy as np
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import std_srvs.srv

# 导入 MCP 工具
sys.path.append('/home/wenhao/uf_custom_ws/src/mtc_tutorial/scripts')
from ros_client_tools import update_cup_pose


class CoordinateTransformNode(Node):
    """坐标转换节点."""

    def __init__(self):
        """初始化节点."""
        super().__init__('coordinate_transform_node')

        # TF2 缓冲区和监听器（增加缓冲区时间）
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 订阅物体检测结果
        self.objects_sub = self.create_subscription(
            ObjectsStamped,
            '/objectsStamped',
            self.objects_callback,
            10
        )

        self.detection_info_sub = self.create_subscription(
            DetectionInfo,
            '/detection_info',
            self.detection_info_callback,
            10
        )

        # 发布转换后的坐标
        self.transformed_pose_pub = self.create_publisher(
            PoseStamped,
            '/object_pose_in_base',
            10
        )

        self.get_logger().info('坐标转换节点已启动')
        self.get_logger().info('等待物体检测结果...')

        # 机器人基坐标系名称（根据实际情况调整）
        self.base_frame = 'link_base'  # 修正为正确的基坐标系名称
        self.camera_frame = 'camera_color_optical_frame'

        # 新增：物体位姿稳定化
        self.pose_filters = {}  # 每个物体一个滤波器
        self.scene_update_config = {
            'enabled': True,
            'min_confidence': 0.8,      # 最小置信度
            'update_interval': 60.0,    # 最大更新间隔（秒）
            'position_threshold': 0.02,  # 位置变化阈值（米）
            'window_size': 15,          # 滑动窗口大小
            'outlier_threshold': 0.05   # 异常值阈值
        }

        # 添加服务：手动确认物体位置
        self.lock_pose_service = self.create_service(
            std_srvs.srv.SetBool, 'lock_object_pose', self.lock_pose_callback)

    def objects_callback(self, msg):
        """处理 2D 物体检测结果."""
        if len(msg.objects.data) < 12:  # 至少需要一个完整的物体信息
            return

        # 解析物体信息：[ObjectId, width, height, h11, h12, h13, h21, h22, h23, h31, h32, h33]
        objects_data = msg.objects.data
        num_objects = len(objects_data) // 12

        self.get_logger().info(f'检测到 {num_objects} 个物体')

        for i in range(num_objects):
            start_idx = i * 12
            object_id = int(objects_data[start_idx])
            width = objects_data[start_idx + 1]
            height = objects_data[start_idx + 2]

            # 齐次变换矩阵 (3x3)
            homography = objects_data[start_idx + 3:start_idx + 12]
            h11, h12, h13 = homography[0:3]
            h21, h22, h23 = homography[3:6]
            h31, h32, h33 = homography[6:9]

            self.get_logger().info(f'物体 ID: {object_id}, 尺寸: {width}x{height}')
            self.get_logger().info(f'中心位置 (像素): ({h13:.1f}, {h23:.1f})')

            # 直接尝试获取物体的 TF 变换（因为detection_info可能没有数据）
            self.get_object_transform(object_id, msg.header.stamp)

    def detection_info_callback(self, msg):
        """处理详细检测信息."""
        if len(msg.ids.data) == 0:
            return

        for i, object_id in enumerate(msg.ids.data):
            self.get_logger().info(f'检测到物体 ID: {object_id.data}')
            self.get_logger().info(f'内点数量: {msg.inliers[i].data}')
            self.get_logger().info(f'外点数量: {msg.outliers[i].data}')

            # 尝试获取物体的 TF 变换（3D 位姿）
            self.get_object_transform(object_id.data, msg.header.stamp)

    def get_object_transform(self, object_id, timestamp):
        """获取物体在相机坐标系中的 TF 变换并转换到基坐标系."""
        object_frame = f'object_{object_id}'

        try:
            # 获取物体相对于相机的变换（使用最新时间戳，更宽松的超时）
            object_to_camera = self.tf_buffer.lookup_transform(
                self.camera_frame,
                object_frame,
                rclpy.time.Time(),  # 使用最新可用的变换
                timeout=Duration(seconds=3.0)
            )

            self.get_logger().info(f'物体 {object_id} 在相机坐标系中的位置:')
            self.get_logger().info(f'  x: {object_to_camera.transform.translation.x:.3f} m')
            self.get_logger().info(f'  y: {object_to_camera.transform.translation.y:.3f} m')
            self.get_logger().info(f'  z: {object_to_camera.transform.translation.z:.3f} m')

            # 转换到机器人基坐标系
            self.transform_to_base_frame(object_to_camera, object_id, timestamp)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'无法获取物体 {object_id} 的 TF 变换: {str(e)}')

    def transform_to_base_frame(self, object_to_camera, object_id, timestamp):
        """将物体坐标转换到机器人基坐标系."""
        try:
            # 获取相机到基坐标系的变换（手眼标定结果，使用最新时间戳）
            camera_to_base = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),  # 使用最新可用的变换
                timeout=Duration(seconds=3.0)
            )

            # 创建物体在相机坐标系中的位姿（使用当前时间戳）
            current_time = self.get_clock().now().to_msg()
            object_pose_camera = PoseStamped()
            object_pose_camera.header.frame_id = self.camera_frame
            object_pose_camera.header.stamp = current_time
            object_pose_camera.pose.position.x = object_to_camera.transform.translation.x
            object_pose_camera.pose.position.y = object_to_camera.transform.translation.y
            object_pose_camera.pose.position.z = object_to_camera.transform.translation.z
            object_pose_camera.pose.orientation = object_to_camera.transform.rotation

            # 转换到基坐标系
            object_pose_base = tf2_geometry_msgs.do_transform_pose(
                object_pose_camera.pose,
                camera_to_base
            )

            # 发布转换后的位姿
            transformed_pose = PoseStamped()
            transformed_pose.header.frame_id = self.base_frame
            transformed_pose.header.stamp = current_time
            transformed_pose.pose = object_pose_base

            self.transformed_pose_pub.publish(transformed_pose)

            self.get_logger().info(f'物体 {object_id} 在基坐标系中的位置:')
            self.get_logger().info(f'  x: {object_pose_base.position.x:.3f} m')
            self.get_logger().info(f'  y: {object_pose_base.position.y:.3f} m')
            self.get_logger().info(f'  z: {object_pose_base.position.z:.3f} m')
            self.get_logger().info('---')

            # 新增：稳定化处理（传递基坐标系中的最终位姿）
            if self.scene_update_config['enabled']:
                self.process_stable_pose_update(object_pose_base, object_id)

            return object_pose_base

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'无法转换到基坐标系: {str(e)}')
            self.get_logger().warn('请确保手眼标定结果正在发布')
            return None

    def get_or_create_filter(self, object_id):
        """获取或创建物体的位姿滤波器."""
        if object_id not in self.pose_filters:
            self.pose_filters[object_id] = ObjectPoseFilter(
                window_size=self.scene_update_config['window_size'],
                position_threshold=self.scene_update_config['position_threshold'],
                update_interval=self.scene_update_config['update_interval']
            )
        return self.pose_filters[object_id]

    def process_stable_pose_update(self, object_pose_base, object_id):
        """处理稳定位姿更新（使用基坐标系中的最终位姿）."""
        try:
            # 获取滤波器
            pose_filter = self.get_or_create_filter(object_id)

            # 创建位姿消息（已经是基坐标系中的位姿）
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.base_frame
            pose_msg.pose = object_pose_base  # 直接使用转换后的基坐标系位姿

            # 添加到滤波器
            stable_pose = pose_filter.add_pose(pose_msg)

            # 如果获得稳定位姿，更新场景
            if stable_pose and stable_pose['confidence'] >= self.scene_update_config['min_confidence']:
                self.update_moveit_scene_stable(stable_pose, object_id)

        except Exception as e:
            self.get_logger().error(f'稳定位姿处理异常: {str(e)}')

    def update_moveit_scene_stable(self, stable_pose, object_id):
        """使用稳定位姿更新MoveIt场景."""
        try:
            pos = stable_pose['position']
            ori = stable_pose['orientation']
            conf = stable_pose['confidence']

            self.get_logger().info(
                f'更新物体 {object_id} 稳定位置到规划场景: '
                f'({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) '
                f'置信度: {conf:.2f}'
            )

            # 调用您的MCP工具
            result = update_cup_pose(
                cup_x=pos[0], cup_y=pos[1], cup_z=pos[2],
                cup_qx=ori[0], cup_qy=ori[1], cup_qz=ori[2], cup_qw=ori[3],
                timeout_sec=3.0
            )

            if result['ok']:
                self.get_logger().info('✅ 规划场景稳定更新成功')
                # 发布可视化标记
                self.publish_confidence_marker(stable_pose, object_id)
            else:
                self.get_logger().warn(f'❌ 规划场景更新失败: {result.get("error", "未知错误")}')

        except Exception as e:
            self.get_logger().error(f'稳定场景更新异常: {str(e)}')

    def publish_confidence_marker(self, stable_pose, object_id):
        """发布置信度可视化标记."""
        try:
            from visualization_msgs.msg import Marker

            # 创建可视化标记
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'stable_objects'
            marker.id = int(object_id)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # 设置位置
            pos = stable_pose['position']
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])

            # 设置方向
            ori = stable_pose['orientation']
            marker.pose.orientation.x = float(ori[0])
            marker.pose.orientation.y = float(ori[1])
            marker.pose.orientation.z = float(ori[2])
            marker.pose.orientation.w = float(ori[3])

            # 设置尺寸（杯子大小）
            marker.scale.x = 0.04  # 直径 4cm
            marker.scale.y = 0.04
            marker.scale.z = 0.10  # 高度 10cm

            # 根据置信度设置颜色（红色->黄色->绿色）
            confidence = stable_pose['confidence']
            if confidence < 0.5:
                # 低置信度：红色
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif confidence < 0.8:
                # 中等置信度：黄色
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                # 高置信度：绿色
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker.color.a = 0.7  # 半透明
            marker.lifetime = Duration(seconds=65.0).to_msg()  # 65秒自动消失

            # 发布标记（需要在__init__中创建发布器）
            if not hasattr(self, 'marker_pub'):
                self.marker_pub = self.create_publisher(Marker, '/stable_object_markers', 10)

            self.marker_pub.publish(marker)

            self.get_logger().debug(f'发布物体 {object_id} 的置信度标记，置信度: {confidence:.2f}')

        except Exception as e:
            self.get_logger().warn(f'发布可视化标记失败: {str(e)}')

    def lock_pose_callback(self, request, response):
        """手动锁定/解锁物体位置服务."""
        for obj_id, pose_filter in self.pose_filters.items():
            pose_filter.locked = request.data

        response.success = True
        response.message = f'物体位置{"锁定" if request.data else "解锁"}成功'
        self.get_logger().info(response.message)
        return response


class ObjectPoseFilter:
    """物体位姿滤波器实现."""

    def __init__(self, window_size=15, position_threshold=0.02, update_interval=60.0):
        """初始化滤波器."""
        self.pose_history = deque(maxlen=window_size)
        self.position_threshold = position_threshold
        self.update_interval = update_interval
        self.last_update_time = 0
        self.stable_pose = None
        self.locked = False

    def add_pose(self, pose_msg):
        """添加位姿数据."""
        if self.locked:
            return self.stable_pose

        current_time = time.time()
        pose_data = {
            'timestamp': current_time,
            'position': np.array([
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z
            ]),
            'orientation': np.array([
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w
            ])
        }

        self.pose_history.append(pose_data)

        # 计算稳定位姿
        if len(self.pose_history) >= 5:  # 至少5个样本
            return self._calculate_stable_pose()

        return None

    def _calculate_stable_pose(self):
        """计算稳定位姿."""
        if not self.pose_history:
            return None

        # 过滤异常值
        filtered_poses = self._filter_outliers()
        if len(filtered_poses) < 3:
            return None

        # 计算加权平均（时间越近权重越大）
        current_time = time.time()
        total_weight = 0
        weighted_pos = np.zeros(3)
        weighted_ori = np.zeros(4)

        for pose in filtered_poses:
            # 时间权重：越新的数据权重越高
            time_diff = current_time - pose['timestamp']
            weight = np.exp(-time_diff / 10.0)  # 10秒衰减

            weighted_pos += weight * pose['position']
            weighted_ori += weight * pose['orientation']
            total_weight += weight

        if total_weight > 0:
            stable_position = weighted_pos / total_weight
            stable_orientation = weighted_ori / total_weight

            # 归一化四元数
            stable_orientation = stable_orientation / np.linalg.norm(stable_orientation)

            # 计算置信度
            confidence = self._calculate_confidence(filtered_poses)

            new_pose = {
                'position': stable_position,
                'orientation': stable_orientation,
                'confidence': confidence,
                'timestamp': current_time
            }

            # 判断是否需要更新
            if self._should_update(new_pose):
                self.stable_pose = new_pose
                self.last_update_time = current_time
                return new_pose

        return self.stable_pose

    def _filter_outliers(self):
        """过滤异常值."""
        if len(self.pose_history) < 3:
            return list(self.pose_history)

        positions = np.array([pose['position'] for pose in self.pose_history])

        # 计算中位数位置
        median_pos = np.median(positions, axis=0)

        # 过滤距离中位数太远的点
        filtered = []
        for pose in self.pose_history:
            distance = np.linalg.norm(pose['position'] - median_pos)
            if distance < 0.1:  # 10cm阈值
                filtered.append(pose)

        return filtered

    def _calculate_confidence(self, poses):
        """计算位姿置信度."""
        if len(poses) < 2:
            return 0.5

        positions = np.array([pose['position'] for pose in poses])
        std_dev = np.std(positions, axis=0)
        avg_std = np.mean(std_dev)

        # 标准差越小，置信度越高
        confidence = np.exp(-avg_std * 50)  # 调整系数
        return min(max(confidence, 0.0), 1.0)

    def _should_update(self, new_pose):
        """判断是否应该更新场景."""
        current_time = time.time()

        # 强制更新条件：超过最大间隔
        if current_time - self.last_update_time > self.update_interval:
            return True

        # 如果没有之前的稳定位姿
        if self.stable_pose is None:
            return True

        # 位置变化阈值检查
        pos_change = np.linalg.norm(new_pose['position'] - self.stable_pose['position'])
        if pos_change > self.position_threshold:
            return True

        # 置信度显著提高
        if new_pose['confidence'] > self.stable_pose['confidence'] + 0.1:
            return True

        return False


def main(args=None):
    """主函数."""
    rclpy.init(args=args)

    node = CoordinateTransformNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 