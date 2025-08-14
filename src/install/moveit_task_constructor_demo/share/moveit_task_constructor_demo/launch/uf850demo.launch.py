import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    add_gripper = LaunchConfiguration('add_gripper', default='true')
    
    # UF850 MoveIt配置
    uf850_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'), 
            'launch', 
            'uf850_moveit_fake.launch.py'
        ])),
        launch_arguments={
            'hw_ns': 'ufactory',
        }.items(),
    )
    
    # MTC演示节点 - 延迟启动以确保MoveIt配置完全加载
    mtc_demo_node = Node(
        package='moveit_task_constructor_demo',
        executable='uf850_pick_place',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ],
    )
    
    # 延迟启动MTC节点
    delayed_mtc_node = TimerAction(
        period=5.0,  # 延迟5秒
        actions=[mtc_demo_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'add_gripper',
            default_value='true',
            description='Whether to add gripper'
        ),
        uf850_moveit_launch,
        delayed_mtc_node,
    ])