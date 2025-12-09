#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    启动 find_object_2d 进行 3D 物体检测
    配置适合深度相机的话题映射
    """
    
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),

        # 启动参数
        DeclareLaunchArgument(
            'gui', 
            default_value='true', 
            description='是否启动图形界面'
        ),
        DeclareLaunchArgument(
            'approx_sync', 
            default_value='true', 
            description='RGB和深度图像近似同步'
        ),
        DeclareLaunchArgument(
            'pnp', 
            default_value='true', 
            description='启用 PnP 算法计算 3D 位姿'
        ),
        DeclareLaunchArgument(
            'object_prefix', 
            default_value='object', 
            description='物体 TF 框架前缀'
        ),
        DeclareLaunchArgument(
            'objects_path', 
            default_value='~/objects', 
            description='物体模型存储目录'
        ),
        DeclareLaunchArgument(
            'settings_path', 
            default_value='~/.ros/find_object_2d.ini', 
            description='配置文件路径'
        ),

        # 相机话题配置 - 根据实际相机话题调整
        DeclareLaunchArgument(
            'rgb_topic', 
            default_value='/camera/camera/color/image_raw', 
            description='RGB 图像话题'
        ),
        DeclareLaunchArgument(
            'depth_topic', 
            default_value='/camera/camera/aligned_depth_to_color/image_raw', 
            description='深度图像话题（对齐到颜色坐标系）'
        ),
        DeclareLaunchArgument(
            'camera_info_topic', 
            default_value='/camera/camera/aligned_depth_to_color/camera_info', 
            description='相机信息话题（对齐深度的相机信息）'
        ),

        # 启动 find_object_2d 节点
        Node(
            package='find_object_2d', 
            executable='find_object_2d', 
            name='find_object_2d',
            output='screen',
            parameters=[{
                'subscribe_depth': True,  # 启用深度订阅
                'gui': LaunchConfiguration('gui'),
                'approx_sync': LaunchConfiguration('approx_sync'),
                'pnp': LaunchConfiguration('pnp'),
                'object_prefix': LaunchConfiguration('object_prefix'),
                'objects_path': LaunchConfiguration('objects_path'),
                'settings_path': LaunchConfiguration('settings_path'),
                # 额外参数
                'objects_path_ini': LaunchConfiguration('objects_path'),
                'session_path': '',
            }],
            remappings=[
                ('rgb/image_rect_color', LaunchConfiguration('rgb_topic')),
                ('depth_registered/image_raw', LaunchConfiguration('depth_topic')),
                ('depth_registered/camera_info', LaunchConfiguration('camera_info_topic'))
            ]
        ), 
    ])

if __name__ == '__main__':
    generate_launch_description() 