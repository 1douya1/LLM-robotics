#!/usr/bin/env python3
"""
计算手眼标定的正确变换：从link_base到camera_link
使用变换复合运算: T_base_to_link = T_base_to_optical * T_optical_to_link
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_to_matrix(quat):
    """将四元数转换为旋转矩阵"""
    # 输入格式：[x, y, z, w]
    return R.from_quat(quat).as_matrix()

def matrix_to_quaternion(matrix):
    """将旋转矩阵转换为四元数"""
    return R.from_matrix(matrix).as_quat()  # 输出格式：[x, y, z, w]

def create_transform_matrix(translation, quaternion):
    """创建4x4变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(quaternion)
    T[:3, 3] = translation
    return T

def print_transform(name, T):
    """打印变换矩阵的信息"""
    translation = T[:3, 3]
    rotation_quat = matrix_to_quaternion(T[:3, :3])
    
    print(f"\n{name}:")
    print(f"  Translation: [{translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}]")
    print(f"  Quaternion (xyzw): [{rotation_quat[0]:.6f}, {rotation_quat[1]:.6f}, {rotation_quat[2]:.6f}, {rotation_quat[3]:.6f}]")
    
    # 转换为RPY角度
    rotation_euler = R.from_quat(rotation_quat).as_euler('xyz', degrees=True)
    print(f"  RPY (degrees): [{rotation_euler[0]:.3f}, {rotation_euler[1]:.3f}, {rotation_euler[2]:.3f}]")

def main():
    # 1. 从手眼标定文件获得的变换：link_base -> camera_color_optical_frame
    base_to_optical_trans = [0.4701272097305125, -0.47598563997501675, 0.4845344093787309]
    base_to_optical_quat = [-0.6123457347988643, -0.6288654378336379, 0.3204295127730189, 0.3562104567751926]
    
    # 2. 从RealSense驱动获得的变换：camera_link -> camera_color_optical_frame
    link_to_optical_trans = [-0.000, 0.015, 0.000]
    link_to_optical_quat = [-0.495, 0.503, -0.496, 0.506]
    
    # 创建变换矩阵
    T_base_to_optical = create_transform_matrix(base_to_optical_trans, base_to_optical_quat)
    T_link_to_optical = create_transform_matrix(link_to_optical_trans, link_to_optical_quat)
    
    print("输入变换:")
    print_transform("link_base -> camera_color_optical_frame (手眼标定)", T_base_to_optical)
    print_transform("camera_link -> camera_color_optical_frame (相机驱动)", T_link_to_optical)
    
    # 计算 camera_color_optical_frame -> camera_link (逆变换)
    T_optical_to_link = np.linalg.inv(T_link_to_optical)
    
    # 计算 link_base -> camera_link = link_base -> optical * optical -> camera_link
    T_base_to_link = T_base_to_optical @ T_optical_to_link
    
    print("\n计算结果:")
    print_transform("camera_color_optical_frame -> camera_link (逆变换)", T_optical_to_link)
    print_transform("link_base -> camera_link (最终结果)", T_base_to_link)
    
    # 输出静态TF发布器命令
    final_trans = T_base_to_link[:3, 3]
    final_quat = matrix_to_quaternion(T_base_to_link[:3, :3])
    final_euler = R.from_quat(final_quat).as_euler('xyz', degrees=False)  # 弧度
    
    print(f"\n=== 静态TF发布器命令 ===")
    print(f"ros2 run tf2_ros static_transform_publisher \\")
    print(f"  {final_trans[0]:.6f} {final_trans[1]:.6f} {final_trans[2]:.6f} \\")
    print(f"  {final_euler[2]:.6f} {final_euler[1]:.6f} {final_euler[0]:.6f} \\")
    print(f"  link_base camera_link")
    
    print(f"\n=== Launch文件参数 ===")
    print(f"bl_x:={final_trans[0]:.6f}")
    print(f"bl_y:={final_trans[1]:.6f}")
    print(f"bl_z:={final_trans[2]:.6f}")
    print(f"bl_roll:={final_euler[0]:.6f}")
    print(f"bl_pitch:={final_euler[1]:.6f}")
    print(f"bl_yaw:={final_euler[2]:.6f}")

if __name__ == '__main__':
    main() 