#!/usr/bin/env python3
"""
Charuco Camera Calibration Script with ROS Support
适用于RealSense相机和ROS话题订阅
"""

import numpy as np
import cv2
import glob
import yaml
import os
from pathlib import Path
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    try:
        from cv_bridge import CvBridge
        CV_BRIDGE_AVAILABLE = True
    except ImportError as e:
        print(f"cv_bridge导入失败: {e}")
        print("可能的解决方案:")
        print("1. pip install 'numpy<2'")
        print("2. pip install opencv-python==4.8.1.78")
        CV_BRIDGE_AVAILABLE = False
    ROS_AVAILABLE = True and CV_BRIDGE_AVAILABLE
    if ROS_AVAILABLE:
        print("检测到ROS2环境和cv_bridge")
    else:
        print("ROS2环境可用但cv_bridge有问题，将使用OpenCV相机接口")
except ImportError:
    ROS_AVAILABLE = False
    CV_BRIDGE_AVAILABLE = False
    print("未检测到ROS2环境，将使用OpenCV相机接口")

class CharucoCalibrator:
    def __init__(self):
        # 根据您的标定板参数设置（通过测试确定为7x9）
        self.CHARUCO_BOARD_SQUARES_X = 7  # 您的标定板是7x9
        self.CHARUCO_BOARD_SQUARES_Y = 9
        self.CHARUCO_SQUARE_LENGTH = 0.025  # 25mm，转换为米
        self.CHARUCO_MARKER_LENGTH = 0.018  # 18mm，转换为米
        
        # 创建Aruco字典和Charuco板
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.charuco_board = cv2.aruco.CharucoBoard(
            (self.CHARUCO_BOARD_SQUARES_X, self.CHARUCO_BOARD_SQUARES_Y),
            self.CHARUCO_SQUARE_LENGTH,
            self.CHARUCO_MARKER_LENGTH,
            self.aruco_dict
        )
        
        # 检测参数
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Charuco检测器（新API）
        try:
            self.charuco_params = cv2.aruco.CharucoParameters()
            self.charuco_detector = cv2.aruco.CharucoDetector(self.charuco_board, self.charuco_params, self.aruco_params)
            self.use_new_api = True
            print("使用新版本Charuco API (OpenCV 4.7+)")
        except AttributeError:
            self.charuco_detector = None
            self.use_new_api = False
            print("使用旧版本Charuco API")
        
        # 存储标定数据
        self.all_charuco_corners = []
        self.all_charuco_ids = []
        self.image_size = None
        
        # ROS相关
        self.current_image = None
        self.image_lock = threading.Lock()
    
    def interpolate_charuco_corners(self, marker_corners, marker_ids, image):
        """
        兼容不同版本OpenCV的Charuco角点插值方法
        """
        if self.use_new_api and self.charuco_detector is not None:
            # 使用新版本API (OpenCV 4.7+)
            try:
                charuco_corners, charuco_ids, marker_corners_detected, marker_ids_detected = self.charuco_detector.detectBoard(image)
                if charuco_corners is not None and charuco_ids is not None:
                    return len(charuco_corners), charuco_corners, charuco_ids
                else:
                    return 0, None, None
            except Exception as e:
                print(f"新API检测失败: {e}")
                return 0, None, None
        else:
            # 尝试旧版本API
            try:
                # 尝试使用旧版本API
                charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, image, self.charuco_board
                )
                return charuco_retval, charuco_corners, charuco_ids
            except (AttributeError, TypeError):
                try:
                    # 尝试带参数的版本
                    charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                        marker_corners, marker_ids, image, self.charuco_board, 
                        cameraMatrix=None, distCoeffs=None
                    )
                    return charuco_retval, charuco_corners, charuco_ids
                except:
                    # 如果还是失败，返回空结果
                    return 0, None, None

class ROSImageSubscriber(Node):
    def __init__(self, calibrator, image_topic):
        super().__init__('charuco_calibrator')
        self.calibrator = calibrator
        
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
        
        print(f"订阅话题: {image_topic}")
        
    def image_callback(self, msg):
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.calibrator.image_lock:
                self.calibrator.current_image = cv_image
                
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
            # 如果是numpy版本问题，给出具体建议
            if '_ARRAY_API' in str(e) or 'AttributeError' in str(e):
                self.get_logger().error('检测到NumPy版本兼容性问题，请运行:')
                self.get_logger().error('pip install "numpy<2" opencv-python==4.8.1.78')

def collect_calibration_images_ros(calibrator, image_topic="/camera/camera/color/image_raw", save_dir="calibration_images"):
    """
    通过ROS话题采集标定图像
    """
    if not ROS_AVAILABLE:
        print("ROS2不可用，请使用OpenCV相机接口")
        return
    
    os.makedirs(save_dir, exist_ok=True)
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建图像订阅节点
    image_subscriber = ROSImageSubscriber(calibrator, image_topic)
    
    # 在单独线程中运行ROS spin
    def ros_spin():
        rclpy.spin(image_subscriber)
    
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True
    ros_thread.start()
    
    image_count = 0
    
    print("RealSense相机标定图像采集:")
    print(f"- 订阅话题: {image_topic}")
    print("- 按 's' 保存当前图像")
    print("- 按 'q' 退出采集")
    print("- 建议采集15-20张不同角度和距离的图像")
    print("- 等待图像数据...")
    
    while True:
        with calibrator.image_lock:
            current_frame = calibrator.current_image
        
        if current_frame is None:
            print("等待ROS图像数据...", end='\r')
            time.sleep(0.1)
            continue
        
        frame = current_frame.copy()
        
        # 检测Aruco标记
        marker_corners, marker_ids, _ = calibrator.detector.detectMarkers(frame)
        
        # 显示检测结果
        display_frame = frame.copy()
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(display_frame, marker_corners, marker_ids)
            
            # 检测Charuco角点
            charuco_retval, charuco_corners, charuco_ids = calibrator.interpolate_charuco_corners(
                marker_corners, marker_ids, frame
            )
            
            if charuco_retval > 0:
                cv2.aruco.drawDetectedCornersCharuco(display_frame, charuco_corners, charuco_ids, (0, 255, 0))
                cv2.putText(display_frame, f"Detected: {charuco_retval} corners", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.putText(display_frame, f"Images saved: {image_count}", 
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(display_frame, "Press 's' to save, 'q' to quit", 
                   (10, display_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(display_frame, f"Topic: {image_topic}", 
                   (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        cv2.imshow('RealSense Charuco Calibration', display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            if marker_ids is not None and len(marker_ids) > 4:  # 至少需要4个标记
                filename = os.path.join(save_dir, f"charuco_{image_count:03d}.jpg")
                cv2.imwrite(filename, frame)
                image_count += 1
                print(f"保存图像: {filename}")
            else:
                print("检测到的标记太少，请调整标定板位置")
        elif key == ord('q'):
            break
    
    cv2.destroyAllWindows()
    image_subscriber.destroy_node()
    rclpy.shutdown()
    print(f"采集完成，共保存 {image_count} 张图像")

def collect_calibration_images_opencv(calibrator, camera_index=0, save_dir="calibration_images"):
    """
    通过OpenCV采集标定图像（原有功能）
    """
    os.makedirs(save_dir, exist_ok=True)
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"无法打开相机 {camera_index}")
        return
    
    image_count = 0
    
    print("OpenCV相机标定图像采集:")
    print("- 按 's' 保存当前图像")
    print("- 按 'q' 退出采集")
    print("- 建议采集15-20张不同角度和距离的图像")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取相机图像")
            break
        
        # 检测Aruco标记
        marker_corners, marker_ids, _ = calibrator.detector.detectMarkers(frame)
        
        # 显示检测结果
        display_frame = frame.copy()
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(display_frame, marker_corners, marker_ids)
            
            # 检测Charuco角点
            charuco_retval, charuco_corners, charuco_ids = calibrator.interpolate_charuco_corners(
                marker_corners, marker_ids, frame
            )
            
            if charuco_retval > 0:
                cv2.aruco.drawDetectedCornersCharuco(display_frame, charuco_corners, charuco_ids, (0, 255, 0))
                cv2.putText(display_frame, f"Detected: {charuco_retval} corners", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.putText(display_frame, f"Images saved: {image_count}", 
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(display_frame, "Press 's' to save, 'q' to quit", 
                   (10, display_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Charuco Calibration', display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            if marker_ids is not None and len(marker_ids) > 4:  # 至少需要4个标记
                filename = os.path.join(save_dir, f"charuco_{image_count:03d}.jpg")
                cv2.imwrite(filename, frame)
                image_count += 1
                print(f"保存图像: {filename}")
            else:
                print("检测到的标记太少，请调整标定板位置")
        elif key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print(f"采集完成，共保存 {image_count} 张图像")

def process_images(calibrator, image_dir="calibration_images"):
    """
    处理标定图像，提取Charuco角点
    """
    image_files = glob.glob(os.path.join(image_dir, "*.jpg"))
    if not image_files:
        print(f"在 {image_dir} 中没有找到图像文件")
        return False
    
    print(f"处理 {len(image_files)} 张标定图像...")
    
    for i, img_path in enumerate(image_files):
        print(f"处理图像 {i+1}/{len(image_files)}: {os.path.basename(img_path)}")
        
        image = cv2.imread(img_path)
        if image is None:
            print(f"无法读取图像: {img_path}")
            continue
            
        if calibrator.image_size is None:
            calibrator.image_size = (image.shape[1], image.shape[0])  # (width, height)
        
        # 转为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 检测Aruco标记
        marker_corners, marker_ids, _ = calibrator.detector.detectMarkers(gray)
        
        if marker_ids is not None and len(marker_ids) > 4:
            # 插值Charuco角点
            charuco_retval, charuco_corners, charuco_ids = calibrator.interpolate_charuco_corners(
                marker_corners, marker_ids, gray
            )
            
            if charuco_retval > 8:  # 至少需要8个角点进行可靠标定
                calibrator.all_charuco_corners.append(charuco_corners)
                calibrator.all_charuco_ids.append(charuco_ids)
                print(f"  检测到 {charuco_retval} 个角点")
            else:
                print(f"  角点太少 ({charuco_retval})，跳过此图像")
        else:
            print("  未检测到足够的Aruco标记，跳过此图像")
    
    print(f"成功处理 {len(calibrator.all_charuco_corners)} 张图像用于标定")
    return len(calibrator.all_charuco_corners) > 0

def calibrate_camera(calibrator):
    """
    执行相机标定
    """
    if len(calibrator.all_charuco_corners) < 10:
        print("标定图像太少，建议至少10张有效图像")
        return False
    
    print("开始相机标定...")
    
    # 执行标定
    try:
        # 尝试新版本API
        if hasattr(cv2, 'calibrateCameraCharuco'):
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCameraCharuco(
                calibrator.all_charuco_corners,
                calibrator.all_charuco_ids,
                calibrator.charuco_board,
                calibrator.image_size,
                None,
                None
            )
        elif hasattr(cv2.aruco, 'calibrateCameraCharuco'):
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                calibrator.all_charuco_corners,
                calibrator.all_charuco_ids,
                calibrator.charuco_board,
                calibrator.image_size,
                None,
                None
            )
        else:
            # 使用普通的棋盘格标定作为后备
            print("使用普通标定作为后备方案...")
            # 转换Charuco角点为标准格式
            object_points = []
            image_points = []
            
            for i in range(len(calibrator.all_charuco_corners)):
                corners = calibrator.all_charuco_corners[i]
                ids = calibrator.all_charuco_ids[i]
                
                if len(corners) >= 8:  # 至少需要8个点
                    # 获取对应的3D点
                    obj_pts = calibrator.charuco_board.getChessboardCorners()[ids.flatten()]
                    object_points.append(obj_pts)
                    image_points.append(corners)
            
            if len(object_points) >= 10:
                ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                    object_points, image_points, calibrator.image_size, None, None
                )
            else:
                raise Exception("标定点不足")
        
        if ret:
            print(f"标定成功! RMS误差: {ret:.4f}")
            
            # 保存标定结果
            save_calibration(calibrator, camera_matrix, dist_coeffs, ret)
            
            # 计算重投影误差
            calculate_reprojection_error(calibrator, camera_matrix, dist_coeffs, rvecs, tvecs)
            
            return True
        else:
            print("标定失败")
            return False
            
    except Exception as e:
        print(f"标定过程中出现错误: {e}")
        return False

def save_calibration(calibrator, camera_matrix, dist_coeffs, rms_error):
    """
    保存标定结果到YAML文件（ROS格式）
    """
    calibration_data = {
        'image_width': int(calibrator.image_size[0]),
        'image_height': int(calibrator.image_size[1]),
        'camera_name': 'realsense_camera',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': camera_matrix.flatten().tolist()
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': dist_coeffs.flatten().tolist()
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': np.eye(3).flatten().tolist()
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': np.hstack([camera_matrix, np.zeros((3, 1))]).flatten().tolist()
        },
        'rms_error': float(rms_error)
    }
    
    # 保存ROS格式的标定文件
    with open('realsense_calibration.yaml', 'w') as f:
        yaml.dump(calibration_data, f, default_flow_style=False)
    
    # 同时保存OpenCV格式（用于其他应用）
    cv_calibration = {
        'camera_matrix': camera_matrix.tolist(),
        'distortion_coefficients': dist_coeffs.tolist(),
        'image_size': list(calibrator.image_size),
        'rms_error': float(rms_error)
    }
    
    with open('realsense_calibration_opencv.yaml', 'w') as f:
        yaml.dump(cv_calibration, f, default_flow_style=False)
    
    print("标定结果已保存到:")
    print("- realsense_calibration.yaml (ROS格式)")
    print("- realsense_calibration_opencv.yaml (OpenCV格式)")
    
    # 打印标定结果
    print("\n=== RealSense相机标定结果 ===")
    print(f"图像尺寸: {calibrator.image_size}")
    print(f"RMS误差: {rms_error:.4f}")
    print(f"相机内参矩阵:\n{camera_matrix}")
    print(f"畸变系数: {dist_coeffs.flatten()}")

def calculate_reprojection_error(calibrator, camera_matrix, dist_coeffs, rvecs, tvecs):
    """
    计算重投影误差
    """
    total_error = 0
    total_points = 0
    
    for i in range(len(calibrator.all_charuco_corners)):
        # 获取3D角点
        charuco_corners_3d = calibrator.charuco_board.getChessboardCorners()[calibrator.all_charuco_ids[i].flatten()]
        
        # 重投影
        projected_points, _ = cv2.projectPoints(
            charuco_corners_3d, rvecs[i], tvecs[i], camera_matrix, dist_coeffs
        )
        
        # 计算误差
        error = cv2.norm(calibrator.all_charuco_corners[i], projected_points, cv2.NORM_L2)
        total_error += error
        total_points += len(calibrator.all_charuco_corners[i])
    
    mean_error = total_error / total_points
    print(f"平均重投影误差: {mean_error:.4f} 像素")

def main():
    calibrator = CharucoCalibrator()
    
    print("RealSense Charuco相机标定程序")
    print("=============================")
    
    while True:
        print("\n请选择操作:")
        if ROS_AVAILABLE:
            print("1. 通过ROS话题采集标定图像 (推荐用于RealSense)")
            print("2. 通过OpenCV采集标定图像")
            print("3. 处理现有图像并标定")
            print("4. 退出")
            max_choice = 4
        else:
            print("1. 通过OpenCV采集标定图像")
            print("2. 处理现有图像并标定")
            print("3. 退出")
            max_choice = 3
        
        try:
            choice = input(f"请输入选择 (1-{max_choice}): ").strip()
            choice = int(choice)
        except ValueError:
            print("无效输入，请输入数字")
            continue
        
        if ROS_AVAILABLE and choice == 1:
            # ROS话题采集
            print("\n常见的RealSense话题:")
            print("- /camera/camera/color/image_raw")
            print("- /camera/color/image_raw") 
            print("- /realsense/color/image_raw")
            
            image_topic = input("请输入图像话题 (默认/camera/camera/color/image_raw): ").strip()
            image_topic = image_topic if image_topic else "/camera/camera/color/image_raw"
            
            save_dir = input("图像保存目录 (默认calibration_images): ").strip()
            save_dir = save_dir if save_dir else "calibration_images"
            
            collect_calibration_images_ros(calibrator, image_topic, save_dir)
            
        elif (ROS_AVAILABLE and choice == 2) or (not ROS_AVAILABLE and choice == 1):
            # OpenCV采集
            camera_index = input("请输入相机索引 (默认0): ").strip()
            camera_index = int(camera_index) if camera_index else 0
            
            save_dir = input("图像保存目录 (默认calibration_images): ").strip()
            save_dir = save_dir if save_dir else "calibration_images"
            
            collect_calibration_images_opencv(calibrator, camera_index, save_dir)
            
        elif (ROS_AVAILABLE and choice == 3) or (not ROS_AVAILABLE and choice == 2):
            # 处理现有图像
            image_dir = input("图像目录 (默认calibration_images): ").strip()
            image_dir = image_dir if image_dir else "calibration_images"
            
            if process_images(calibrator, image_dir):
                calibrate_camera(calibrator)
            
        elif (ROS_AVAILABLE and choice == 4) or (not ROS_AVAILABLE and choice == 3):
            break
        else:
            print("无效选择，请重试")

if __name__ == "__main__":
    main()