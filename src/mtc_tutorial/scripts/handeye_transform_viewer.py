#!/usr/bin/env python3
"""
手眼标定变换查看器
实时查看相机到机器人基座的变换关系
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from rclpy.time import Time
import math

class HandeyeTransformViewer(Node):
    def __init__(self):
        super().__init__('handeye_transform_viewer')
        
        # 创建TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 创建定时器，每秒检查一次变换
        self.timer = self.create_timer(1.0, self.check_transform)
        
        # 配置参数
        self.robot_base_frame = 'link_base'
        self.camera_frame = 'camera_color_optical_frame'
        
        self.get_logger().info(f'开始监听从 {self.robot_base_frame} 到 {self.camera_frame} 的变换...')
    
    def check_transform(self):
        """检查并显示变换关系"""
        try:
            # 获取当前时间
            now = Time()
            
            # 查找变换
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,      # 目标坐标系
                self.camera_frame,          # 源坐标系
                now
            )
            
            # 提取变换信息
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # 计算欧拉角（更容易理解）
            euler_angles = self.quaternion_to_euler(rotation.x, rotation.y, rotation.z, rotation.w)
            
            # 显示变换结果
            self.get_logger().info('=' * 60)
            self.get_logger().info('手眼标定变换结果:')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'从: {self.robot_base_frame}')
            self.get_logger().info(f'到: {self.camera_frame}')
            self.get_logger().info('')
            self.get_logger().info('平移 (米):')
            self.get_logger().info(f'  X: {translation.x:8.4f}')
            self.get_logger().info(f'  Y: {translation.y:8.4f}')
            self.get_logger().info(f'  Z: {translation.z:8.4f}')
            self.get_logger().info('')
            self.get_logger().info('旋转 (四元数):')
            self.get_logger().info(f'  X: {rotation.x:8.4f}')
            self.get_logger().info(f'  Y: {rotation.y:8.4f}')
            self.get_logger().info(f'  Z: {rotation.z:8.4f}')
            self.get_logger().info(f'  W: {rotation.w:8.4f}')
            self.get_logger().info('')
            self.get_logger().info('旋转 (欧拉角, 度):')
            self.get_logger().info(f'  Roll (绕X轴):  {euler_angles[0]:8.2f}°')
            self.get_logger().info(f'  Pitch (绕Y轴): {euler_angles[1]:8.2f}°')
            self.get_logger().info(f'  Yaw (绕Z轴):   {euler_angles[2]:8.2f}°')
            self.get_logger().info('')
            
            # 计算距离
            distance = math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)
            self.get_logger().info(f'相机到机器人基座的距离: {distance:8.4f} 米')
            self.get_logger().info('=' * 60)
            
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'无法获取变换: {ex}')
            self.get_logger().info('请确保:')
            self.get_logger().info('1. 手眼标定已完成并保存')
            self.get_logger().info('2. 已启动 publish.launch.py')
            self.get_logger().info('3. 标定名称正确')
    
    def quaternion_to_euler(self, x, y, z, w):
        """将四元数转换为欧拉角（度）"""
        # 绕X轴旋转 (Roll)
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        
        # 绕Y轴旋转 (Pitch)
        pitch = math.asin(2 * (w * y - z * x))
        
        # 绕Z轴旋转 (Yaw)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        
        # 转换为度
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        return roll_deg, pitch_deg, yaw_deg

def main(args=None):
    rclpy.init(args=args)
    
    viewer = HandeyeTransformViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()