#!/usr/bin/env python3
"""
RealSense 相机杯子照片采集脚本
功能: 连接RealSense相机，实时预览，按's'键保存杯子照片
作者: AI Assistant
"""

import cv2
import numpy as np
import os
import time
from datetime import datetime
from pathlib import Path
import threading

# 尝试导入ROS2相关模块
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    try:
        from cv_bridge import CvBridge
        CV_BRIDGE_AVAILABLE = True
    except ImportError as e:
        print(f"cv_bridge导入失败: {e}")
        print("建议解决方案:")
        print("1. pip install 'numpy<2'")
        print("2. pip install opencv-python==4.8.1.78")
        CV_BRIDGE_AVAILABLE = False
    ROS_AVAILABLE = True and CV_BRIDGE_AVAILABLE
except ImportError:
    ROS_AVAILABLE = False
    CV_BRIDGE_AVAILABLE = False
    print("未检测到ROS2环境，将使用OpenCV相机接口")

class CupPhotoCapture:
    def __init__(self, save_dir="cup_photos"):
        """
        初始化杯子照片采集器
        """
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(exist_ok=True)
        
        # 图像数据
        self.current_image = None
        self.image_lock = threading.Lock()
        
        # 计数器
        self.photo_count = 0
        
        print(f"照片保存目录: {self.save_dir.absolute()}")
        
    def save_photo(self, image):
        """
        保存照片到指定目录
        """
        if image is None:
            print("无图像数据可保存!")
            return False
            
        # 生成文件名（包含时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"cup_{timestamp}_{self.photo_count:03d}.jpg"
        filepath = self.save_dir / filename
        
        # 保存图像
        success = cv2.imwrite(str(filepath), image)
        if success:
            self.photo_count += 1
            print(f"✓ 照片已保存: {filename}")
            return True
        else:
            print(f"✗ 保存失败: {filename}")
            return False

class ROSCameraSubscriber(Node):
    """
    ROS2图像话题订阅器
    """
    def __init__(self, capture_handler, image_topic="/camera/camera/color/image_raw"):
        super().__init__('cup_photo_subscriber')
        self.capture_handler = capture_handler
        
        if not CV_BRIDGE_AVAILABLE:
            raise RuntimeError("cv_bridge不可用，无法创建ROS图像订阅器")
            
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        print(f"已订阅ROS话题: {image_topic}")
        
    def image_callback(self, msg):
        """
        图像话题回调函数
        """
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.capture_handler.image_lock:
                self.capture_handler.current_image = cv_image
                
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')

def capture_with_ros(image_topic="/camera/camera/color/image_raw", save_dir="cup_photos"):
    """
    使用ROS2话题采集杯子照片
    """
    if not ROS_AVAILABLE:
        print("ROS2不可用，请使用OpenCV相机接口")
        return False
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建采集器
    capture_handler = CupPhotoCapture(save_dir)
    
    # 创建图像订阅节点
    image_subscriber = ROSCameraSubscriber(capture_handler, image_topic)
    
    # 在单独线程中运行ROS spin
    def ros_spin():
        rclpy.spin(image_subscriber)
    
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True
    ros_thread.start()
    
    print("\n=== RealSense 杯子照片采集 ===")
    print(f"图像话题: {image_topic}")
    print(f"保存目录: {save_dir}")
    print("\n操作说明:")
    print("- 实时预览相机画面")
    print("- 按 's' 键保存当前照片")
    print("- 按 'q' 键退出程序")
    print("- 按 'c' 键清除画面统计信息")
    print("\n等待图像数据...")
    
    try:
        while True:
            with capture_handler.image_lock:
                current_frame = capture_handler.current_image
            
            if current_frame is None:
                print("等待ROS图像数据...", end='\r')
                time.sleep(0.1)
                continue
            
            # 创建显示画面副本
            display_frame = current_frame.copy()
            
            # 添加状态信息
            cv2.putText(display_frame, f"Photos saved: {capture_handler.photo_count}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display_frame, "Press 's' to save, 'q' to quit", 
                       (10, display_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, f"Topic: {image_topic}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # 显示图像
            cv2.imshow('RealSense Cup Photo Capture', display_frame)
            
            # 处理按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') or key == ord('S'):
                capture_handler.save_photo(current_frame)
            elif key == ord('q') or key == ord('Q'):
                print("\n退出程序...")
                break
            elif key == ord('c') or key == ord('C'):
                print("\n清除统计信息")
                capture_handler.photo_count = 0
    
    except KeyboardInterrupt:
        print("\n用户中断，程序退出")
    
    finally:
        cv2.destroyAllWindows()
        image_subscriber.destroy_node()
        rclpy.shutdown()
        print(f"采集完成，共保存 {capture_handler.photo_count} 张照片")
        return True

def capture_with_opencv(camera_index=0, save_dir="cup_photos"):
    """
    使用OpenCV直接连接相机采集杯子照片
    """
    # 创建采集器
    capture_handler = CupPhotoCapture(save_dir)
    
    # 打开相机
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"无法打开相机设备 {camera_index}")
        return False
    
    # 设置相机参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    print("\n=== OpenCV 杯子照片采集 ===")
    print(f"相机设备: {camera_index}")
    print(f"保存目录: {save_dir}")
    print("\n操作说明:")
    print("- 实时预览相机画面")
    print("- 按 's' 键保存当前照片")
    print("- 按 'q' 键退出程序")
    print("- 按 'c' 键清除画面统计信息")
    print("\n开始采集...")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法获取相机画面")
                break
            
            # 创建显示画面副本
            display_frame = frame.copy()
            
            # 添加状态信息
            cv2.putText(display_frame, f"Photos saved: {capture_handler.photo_count}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display_frame, "Press 's' to save, 'q' to quit", 
                       (10, display_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, f"Camera: {camera_index}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # 显示图像
            cv2.imshow('OpenCV Cup Photo Capture', display_frame)
            
            # 处理按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') or key == ord('S'):
                capture_handler.save_photo(frame)
            elif key == ord('q') or key == ord('Q'):
                print("\n退出程序...")
                break
            elif key == ord('c') or key == ord('C'):
                print("\n清除统计信息")
                capture_handler.photo_count = 0
    
    except KeyboardInterrupt:
        print("\n用户中断，程序退出")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"采集完成，共保存 {capture_handler.photo_count} 张照片")
        return True

def list_camera_devices():
    """
    列出可用的相机设备
    """
    print("检测可用相机设备...")
    available_cameras = []
    
    for i in range(10):  # 检测前10个设备
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            available_cameras.append(i)
            # 获取相机信息
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            fps = cap.get(cv2.CAP_PROP_FPS)
            print(f"相机 {i}: {int(width)}x{int(height)} @ {fps}fps")
            cap.release()
    
    if not available_cameras:
        print("未检测到可用相机设备")
    else:
        print(f"检测到 {len(available_cameras)} 个相机设备: {available_cameras}")
    
    return available_cameras

def main():
    """
    主函数
    """
    print("RealSense 杯子照片采集脚本")
    print("=" * 40)
    
    # 检查参数
    import argparse
    parser = argparse.ArgumentParser(description='RealSense杯子照片采集')
    parser.add_argument('--mode', choices=['ros', 'opencv'], default='ros',
                       help='采集模式: ros(ROS话题) 或 opencv(直接相机)')
    parser.add_argument('--topic', default='/camera/camera/color/image_raw',
                       help='ROS图像话题 (仅ros模式)')
    parser.add_argument('--camera', type=int, default=0,
                       help='相机设备索引 (仅opencv模式)')
    parser.add_argument('--output', default='cup_photos',
                       help='照片保存目录')
    parser.add_argument('--list-cameras', action='store_true',
                       help='列出可用相机设备')
    
    args = parser.parse_args()
    
    if args.list_cameras:
        list_camera_devices()
        return
    
    # 根据模式选择采集方式
    if args.mode == 'ros':
        if ROS_AVAILABLE:
            success = capture_with_ros(args.topic, args.output)
        else:
            print("ROS2环境不可用，自动切换到OpenCV模式")
            success = capture_with_opencv(args.camera, args.output)
    else:
        success = capture_with_opencv(args.camera, args.output)
    
    if success:
        print("程序正常结束")
    else:
        print("程序异常结束")

if __name__ == "__main__":
    main() 