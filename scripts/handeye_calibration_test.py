#!/usr/bin/env python3
"""
手眼标定精度验证系统
使用Charuco板验证手眼标定的准确性
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import numpy as np
import math

class HandEyeValidationNode(Node):
    def __init__(self):
        super().__init__('handeye_validation')
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 订阅find_object_2d检测结果
        self.objects_sub = self.create_subscription(
            Float32MultiArray,
            '/objects',
            self.objects_callback,
            10
        )
        
        # 发布验证结果
        self.validation_pose_pub = self.create_publisher(
            PoseStamped,
            '/validation_object_pose_robot',
            10
        )
        
        self.validation_marker_pub = self.create_publisher(
            Marker,
            '/validation_marker',
            10
        )
        
        # 存储检测结果用于精度分析
        self.detection_history = []
        self.max_history_size = 50
        
        self.get_logger().info("手眼标定验证节点已启动")
        self.get_logger().info("等待find_object_2d检测结果...")
        self.get_logger().info("请将Charuco板放置在不同位置进行验证")
        
        # 创建定时器定期输出统计信息
        self.stats_timer = self.create_timer(5.0, self.print_statistics)
    
    def objects_callback(self, msg):
        """处理检测到的物体"""
        if len(msg.data) < 12:  # 需要至少一个完整物体的数据
            return
        
        # 解析第一个检测到的物体
        data = msg.data
        object_id = int(data[0])
        
        # 四元数和位置（相机坐标系）
        qx, qy, qz, qw = data[3:7]
        tx, ty, tz = data[7:10]
        
        # 创建相机坐标系下的位姿
        pose_camera = PoseStamped()
        pose_camera.header.frame_id = 'camera_color_optical_frame'
        pose_camera.header.stamp = self.get_clock().now().to_msg()
        
        pose_camera.pose.position.x = tx
        pose_camera.pose.position.y = ty
        pose_camera.pose.position.z = tz
        
        pose_camera.pose.orientation.x = qx
        pose_camera.pose.orientation.y = qy
        pose_camera.pose.orientation.z = qz
        pose_camera.pose.orientation.w = qw
        
        # 转换到机器人基坐标系
        try:
            # 查找手眼标定变换
            transform = self.tf_buffer.lookup_transform(
                'link_base',  # 目标坐标系
                'camera_color_optical_frame',  # 源坐标系
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 应用变换
            pose_robot = do_transform_pose(pose_camera.pose, transform)
            
            # 发布机器人坐标系下的位姿
            pose_robot_stamped = PoseStamped()
            pose_robot_stamped.header.frame_id = 'link_base'
            pose_robot_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_robot_stamped.pose = pose_robot
            
            self.validation_pose_pub.publish(pose_robot_stamped)
            
            # 记录检测历史
            detection_data = {
                'timestamp': self.get_clock().now().nanoseconds / 1e9,
                'camera_pose': {
                    'x': tx, 'y': ty, 'z': tz,
                    'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
                },
                'robot_pose': {
                    'x': pose_robot.position.x,
                    'y': pose_robot.position.y, 
                    'z': pose_robot.position.z,
                    'qx': pose_robot.orientation.x,
                    'qy': pose_robot.orientation.y,
                    'qz': pose_robot.orientation.z,
                    'qw': pose_robot.orientation.w
                }
            }
            
            self.detection_history.append(detection_data)
            if len(self.detection_history) > self.max_history_size:
                self.detection_history.pop(0)
            
            # 创建可视化标记
            self.publish_validation_marker(pose_robot_stamped)
            
            # 输出当前检测结果
            self.get_logger().info(
                f"检测到Charuco板 - 机器人坐标系: "
                f"位置=({pose_robot.position.x:.3f}, {pose_robot.position.y:.3f}, {pose_robot.position.z:.3f}) "
                f"距离={math.sqrt(pose_robot.position.x**2 + pose_robot.position.y**2 + pose_robot.position.z**2):.3f}m"
            )
            
            # 实时精度分析
            if len(self.detection_history) >= 10:
                self.analyze_detection_stability()
                
        except Exception as e:
            self.get_logger().warn(f"坐标变换失败: {e}")
    
    def publish_validation_marker(self, pose_stamped):
        """发布验证标记"""
        marker = Marker()
        marker.header = pose_stamped.header
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 使用检测到的位姿
        marker.pose = pose_stamped.pose
        
        # Charuco板尺寸 (根据您的7x9板子估计)
        marker.scale.x = 0.18  # 大约18cm宽
        marker.scale.y = 0.22  # 大约22cm高  
        marker.scale.z = 0.005 # 5mm厚
        
        # 验证颜色（蓝色）
        marker.color.r = 0.2
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        # 持续时间
        marker.lifetime.sec = 2
        
        self.validation_marker_pub.publish(marker)
    
    def analyze_detection_stability(self):
        """分析检测稳定性"""
        if len(self.detection_history) < 10:
            return
        
        recent_detections = self.detection_history[-10:]  # 最近10次检测
        
        # 计算位置方差
        positions = np.array([
            [d['robot_pose']['x'], d['robot_pose']['y'], d['robot_pose']['z']]
            for d in recent_detections
        ])
        
        position_std = np.std(positions, axis=0)
        position_mean = np.mean(positions, axis=0)
        
        # 计算总体位置稳定性
        total_std = np.linalg.norm(position_std)
        
        # 输出稳定性分析
        if total_std < 0.005:  # 5mm
            stability = "优秀"
        elif total_std < 0.01:  # 10mm
            stability = "良好"
        elif total_std < 0.02:  # 20mm
            stability = "可接受"
        else:
            stability = "需要改进"
        
        self.get_logger().info(
            f"检测稳定性: {stability} | "
            f"位置标准差: X={position_std[0]*1000:.1f}mm, "
            f"Y={position_std[1]*1000:.1f}mm, Z={position_std[2]*1000:.1f}mm | "
            f"总体={total_std*1000:.1f}mm"
        )
    
    def print_statistics(self):
        """定期打印统计信息"""
        if len(self.detection_history) < 5:
            return
        
        recent_detections = self.detection_history[-20:] if len(self.detection_history) >= 20 else self.detection_history
        
        positions = np.array([
            [d['robot_pose']['x'], d['robot_pose']['y'], d['robot_pose']['z']]
            for d in recent_detections
        ])
        
        position_mean = np.mean(positions, axis=0)
        position_std = np.std(positions, axis=0)
        
        self.get_logger().info("=== 手眼标定验证统计 ===")
        self.get_logger().info(f"样本数量: {len(recent_detections)}")
        self.get_logger().info(
            f"平均位置: ({position_mean[0]:.3f}, {position_mean[1]:.3f}, {position_mean[2]:.3f})"
        )
        self.get_logger().info(
            f"位置精度: ±{position_std[0]*1000:.1f}mm, ±{position_std[1]*1000:.1f}mm, ±{position_std[2]*1000:.1f}mm"
        )
        
        # 给出精度评估
        total_error = np.linalg.norm(position_std)
        if total_error < 0.005:
            grade = "A+ (专业级)"
        elif total_error < 0.01:
            grade = "A (优秀)"
        elif total_error < 0.02:
            grade = "B (良好)"
        elif total_error < 0.05:
            grade = "C (可接受)"
        else:
            grade = "D (需要重新标定)"
        
        self.get_logger().info(f"标定精度等级: {grade}")
        self.get_logger().info("=" * 35)

def main():
    rclpy.init()
    
    validator = HandEyeValidationNode()
    
    print("\\n=== 手眼标定精度验证 ===")
    print("使用说明:")
    print("1. 将Charuco板放在不同位置")
    print("2. 观察检测精度和稳定性")
    print("3. 在RViz中查看可视化结果")
    print("4. 按Ctrl+C退出")
    print()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        print("\\n验证完成")
    
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()