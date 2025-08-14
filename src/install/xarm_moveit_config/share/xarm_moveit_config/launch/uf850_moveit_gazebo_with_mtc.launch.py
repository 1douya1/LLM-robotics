#!/usr/bin/env python3
# 基于原有的uf850_moveit_gazebo.launch.py，添加MTC支持和RViz

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    # 所有原有参数保持不变
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='uf850')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')
    
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    show_rviz = LaunchConfiguration('show_rviz', default=True)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = 'gazebo_ros2_control/GazeboSystem'
    controllers_name = 'fake_controllers'

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}{}_controllers.yaml'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        update_rate=1000,
        use_sim_time=True,
        robot_type=robot_type.perform(context)
    )

    moveit_config = MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        dof=dof,
        robot_type=robot_type,
        prefix=prefix,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=attach_to,
        attach_xyz=attach_xyz,
        attach_rpy=attach_rpy,
        mesh_suffix=mesh_suffix,
        kinematics_suffix=kinematics_suffix,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    ).to_moveit_configs()

    moveit_config_dump = yaml.dump(moveit_config.to_dict())

    # 启动Gazebo（原有的方式）
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_gazebo'), 'launch', '_robot_beside_table_gazebo.launch.py'])),
        launch_arguments={
            'dof': dof,
            'robot_type': robot_type,
            'prefix': prefix,
            'moveit_config_dump': moveit_config_dump,
            'load_controller': 'true',
            'show_rviz': 'false',  # 我们用自己的RViz
            'no_gui_ctrl': no_gui_ctrl,
        }.items(),
    )

    # 添加MTC capability的move_group节点
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            # 关键：添加ExecuteTaskSolutionCapability
            {'capabilities': 'move_group/ExecuteTaskSolutionCapability'}
        ],
    )

    # 添加Static TF
    xyz = attach_xyz.perform(context)[1:-1].split(' ')
    rpy = attach_rpy.perform(context)[1:-1].split(' ')
    args = xyz + rpy + [attach_to.perform(context), '{}link_base'.format(prefix.perform(context))]
    
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=args
    )

    # 添加RViz节点 - 尝试使用MTC配置，如果不存在则使用xarm的moveit配置
    try:
        # 首先尝试使用MTC专用配置
        rviz_config_file = os.path.join(
            get_package_share_directory("moveit_task_constructor_demo"), 
            "config", "Mtc.rviz"
        )
        if not os.path.exists(rviz_config_file):
            raise FileNotFoundError
    except:
        # 如果MTC配置不存在，使用xarm的moveit配置
        rviz_config_file = os.path.join(
            get_package_share_directory("xarm_moveit_config"), 
            "rviz", "moveit.rviz"
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': moveit_config.to_dict()['robot_description'],
                'robot_description_semantic': moveit_config.to_dict()['robot_description_semantic'],
                'robot_description_kinematics': moveit_config.to_dict()['robot_description_kinematics'],
                'robot_description_planning': moveit_config.to_dict()['robot_description_planning'],
                'planning_pipelines': moveit_config.to_dict()['planning_pipelines'],
                'use_sim_time': True
            }
        ],
        condition=IfCondition(show_rviz),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    return [
        robot_gazebo_launch,
        static_tf,
        move_group_node,
        rviz_node,  # 添加RViz
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('robot_type', default_value='uf850'),
        DeclareLaunchArgument('add_gripper', default_value='true'),
        DeclareLaunchArgument('hw_ns', default_value='ufactory'),
        DeclareLaunchArgument('show_rviz', default_value='true', description='Whether to start RViz'),
        OpaqueFunction(function=launch_setup)
    ]) 