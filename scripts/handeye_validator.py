#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import time

class CharucoHandEyeValidator(Node):
    def __init__(self):
        super().__init__('charuco_handeye_validator')
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 存储验证数据
        self.camera_transforms = []
        self.robot_poses = []
        
        # 定时器，每2秒检查一次变换
        self.timer = self.create_timer(2.0, self.validate_transforms)
        
        self.get_logger().info('手眼标定验证器已启动...')
        self.get_logger().info('监控坐标系变换: camera_color_optical_frame -> link_base')
        self.get_logger().info('正在验证手眼标定的准确性...')
        
    def validate_transforms(self):
        try:
            # 获取相机到机器人基座的变换（手眼标定结果）
            camera_to_base = self.tf_buffer.lookup_transform(
                'link_base',  # 目标坐标系（机器人基座）
                'camera_color_optical_frame',  # 源坐标系（相机）
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 获取末端执行器到基座的变换（机器人当前位姿）
            ee_to_base = self.tf_buffer.lookup_transform(
                'link_base',  # 目标坐标系
                'link6',  # 末端执行器坐标系
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 记录数据
            current_time = time.time()
            
            camera_data = {
                'timestamp': current_time,
                'x': camera_to_base.transform.translation.x,
                'y': camera_to_base.transform.translation.y,
                'z': camera_to_base.transform.translation.z,
                'qx': camera_to_base.transform.rotation.x,
                'qy': camera_to_base.transform.rotation.y,
                'qz': camera_to_base.transform.rotation.z,
                'qw': camera_to_base.transform.rotation.w
            }
            
            robot_data = {
                'timestamp': current_time,
                'x': ee_to_base.transform.translation.x,
                'y': ee_to_base.transform.translation.y,
                'z': ee_to_base.transform.translation.z,
                'qx': ee_to_base.transform.rotation.x,
                'qy': ee_to_base.transform.rotation.y,
                'qz': ee_to_base.transform.rotation.z,
                'qw': ee_to_base.transform.rotation.w
            }
            
            self.camera_transforms.append(camera_data)
            self.robot_poses.append(robot_data)
            
            # 显示当前变换
            self.get_logger().info('=== 当前变换状态 ===')
            self.get_logger().info(f'相机->基座: X={camera_data["x"]:.3f}, Y={camera_data["y"]:.3f}, Z={camera_data["z"]:.3f}')
            self.get_logger().info(f'末端->基座: X={robot_data["x"]:.3f}, Y={robot_data["y"]:.3f}, Z={robot_data["z"]:.3f}')
            
            # 分析稳定性（需要至少3个数据点）
            if len(self.camera_transforms) >= 3:
                self.analyze_transform_stability()
                 
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(f'无法获取变换: {ex}')
    
    def analyze_transform_stability(self):
        """分析手眼标定变换的稳定性"""
        if len(self.camera_transforms) >= 3:
            # 分析相机变换稳定性
            recent_camera = self.camera_transforms[-3:]
            camera_positions = np.array([[t['x'], t['y'], t['z']] for t in recent_camera])
            camera_std = np.std(camera_positions, axis=0)
            
            self.get_logger().info('=== 手眼标定稳定性分析 ===')
            self.get_logger().info(f'相机位置标准差: X={camera_std[0]:.4f}m, Y={camera_std[1]:.4f}m, Z={camera_std[2]:.4f}m')
            
            # 评估精度
            if np.all(camera_std < 0.001):  # 1mm
                self.get_logger().info('✅ 手眼标定精度: 优秀 (<1mm)')
            elif np.all(camera_std < 0.005):  # 5mm
                self.get_logger().info('✅ 手眼标定精度: 良好 (<5mm)')
            elif np.all(camera_std < 0.01):  # 1cm
                self.get_logger().info('⚠️ 手眼标定精度: 一般 (<1cm)')
            else:
                self.get_logger().info('❌ 手眼标定精度: 需要改进 (>1cm)')
            
            # 计算总体偏移量
            avg_position = np.mean(camera_positions, axis=0)
            self.get_logger().info(f'相机平均位置: X={avg_position[0]:.3f}, Y={avg_position[1]:.3f}, Z={avg_position[2]:.3f}')
    
    def validate_specific_position(self, target_x, target_y, target_z):
        """验证特定位置的准确性（可以手动调用）"""
        try:
            camera_to_base = self.tf_buffer.lookup_transform(
                'link_base',
                'camera_color_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 计算与期望位置的偏差
            actual_x = camera_to_base.transform.translation.x
            actual_y = camera_to_base.transform.translation.y
            actual_z = camera_to_base.transform.translation.z
            
            error_x = abs(actual_x - target_x)
            error_y = abs(actual_y - target_y)
            error_z = abs(actual_z - target_z)
            
            total_error = np.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            self.get_logger().info('=== 位置精度验证 ===')
            self.get_logger().info(f'期望位置: X={target_x:.3f}, Y={target_y:.3f}, Z={target_z:.3f}')
            self.get_logger().info(f'实际位置: X={actual_x:.3f}, Y={actual_y:.3f}, Z={actual_z:.3f}')
            self.get_logger().info(f'位置误差: X={error_x:.3f}, Y={error_y:.3f}, Z={error_z:.3f}')
            self.get_logger().info(f'总体误差: {total_error:.3f}m')
            
            if total_error < 0.01:  # 1cm
                self.get_logger().info('✅ 位置精度: 优秀')
            elif total_error < 0.02:  # 2cm
                self.get_logger().info('✅ 位置精度: 良好')
            elif total_error < 0.05:  # 5cm
                self.get_logger().info('⚠️ 位置精度: 一般')
            else:
                self.get_logger().info('❌ 位置精度: 需要改进')
                 
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().error(f'验证失败: {ex}')

def main():
    rclpy.init()
    node = CharucoHandEyeValidator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('验证器停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 