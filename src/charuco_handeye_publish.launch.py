#!/usr/bin/env python3
"""
Charuco手眼标定结果发布Launch文件
在生产环境中发布已标定好的手眼变换关系
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 参数声明
    calibration_name_arg = DeclareLaunchArgument(
        'calibration_name',
        default_value='charuco_eye_to_hand_1',
        description='手眼标定的名称标识符'
    )
    
    # 图像话题参数（可选，用于验证）
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw',
        description='RealSense相机图像话题'
    )
    
    # 相机frame参数
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description='相机光学坐标系frame'
    )
    
    # Charuco板参数（用于验证）
    charuco_frame_arg = DeclareLaunchArgument(
        'charuco_frame',
        default_value='charuco_board',
        description='Charuco标定板坐标系'
    )
    
    # 标定文件路径
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='realsense_calibration_opencv.yaml',
        description='相机标定文件路径'
    )
    
    # 是否启用Charuco位姿发布器（用于验证标定结果）
    enable_charuco_publisher_arg = DeclareLaunchArgument(
        'enable_charuco_publisher',
        default_value='false',
        description='是否启用Charuco位姿发布器用于验证'
    )

    # 是否采用静态外参(生产推荐)：发布 link_base -> camera_link
    use_static_extrinsics_arg = DeclareLaunchArgument(
        'use_static_extrinsics',
        default_value='false',
        description='使用静态TF发布link_base到camera_link的外参（推荐）'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='link_base',
        description='机器人基座坐标系（规划根）'
    )
    camera_root_frame_arg = DeclareLaunchArgument(
        'camera_root_frame',
        default_value='camera_link',
        description='相机根坐标系（建议camera_link）'
    )

    # 外参六自由度（米/弧度）
    bl_x_arg = DeclareLaunchArgument('bl_x', default_value='0.0')
    bl_y_arg = DeclareLaunchArgument('bl_y', default_value='0.0')
    bl_z_arg = DeclareLaunchArgument('bl_z', default_value='0.0')
    bl_roll_arg  = DeclareLaunchArgument('bl_roll',  default_value='0.0')
    bl_pitch_arg = DeclareLaunchArgument('bl_pitch', default_value='0.0')
    bl_yaw_arg   = DeclareLaunchArgument('bl_yaw',   default_value='0.0')
    
    # easy_handeye2结果发布（仅在未使用静态外参时）
    easy_handeye2_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('easy_handeye2'),
                'launch',
                'publish.launch.py'
            ])
        ]),
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_static_extrinsics'))
    )
    
    # 静态TF：link_base -> camera_link（推荐）
    base_to_camera_static = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[
            LaunchConfiguration('bl_x'),
            LaunchConfiguration('bl_y'),
            LaunchConfiguration('bl_z'),
            # ROS 2 顺序：yaw, pitch, roll
            LaunchConfiguration('bl_yaw'),
            LaunchConfiguration('bl_pitch'),
            LaunchConfiguration('bl_roll'),
            LaunchConfiguration('base_frame'),
            LaunchConfiguration('camera_root_frame')
        ],
        name='base_to_camera_static',
        output='screen',
        respawn=True,
        condition=IfCondition(LaunchConfiguration('use_static_extrinsics'))
    )
    
    # 可选的Charuco位姿发布器（用于验证标定结果）
    charuco_pose_publisher = Node(
        package='mtc_tutorial',
        executable='charuco_pose_publisher.py',
        name='charuco_pose_publisher_verify',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'marker_frame': LaunchConfiguration('charuco_frame'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'board_size_x': 7,
            'board_size_y': 9,
            'square_length': 0.025,
            'marker_length': 0.018,
        }],
        condition=IfCondition(LaunchConfiguration('enable_charuco_publisher')),
        output='screen'
    )
    
    return LaunchDescription([
        # 参数声明
        calibration_name_arg,
        image_topic_arg,
        camera_frame_arg,
        charuco_frame_arg,
        calibration_file_arg,
        enable_charuco_publisher_arg,
        use_static_extrinsics_arg,
        base_frame_arg,
        camera_root_frame_arg,
        bl_x_arg, bl_y_arg, bl_z_arg,
        bl_roll_arg, bl_pitch_arg, bl_yaw_arg,
        
        # 节点启动
        easy_handeye2_publisher,
        base_to_camera_static,
        charuco_pose_publisher,
    ]) 