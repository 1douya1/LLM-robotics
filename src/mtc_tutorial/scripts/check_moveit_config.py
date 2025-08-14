#!/usr/bin/env python3
"""
检查MoveIt配置是否正确加载的脚本
用于验证move_group是否包含所有必要的参数
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, ListParameters
import yaml

class MoveItConfigChecker(Node):
    def __init__(self):
        super().__init__('moveit_config_checker')
        
        self.check_configs()
    
    def check_configs(self):
        """检查关键的MoveIt配置参数"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('MoveIt Configuration Check for UF850')
        self.get_logger().info('=' * 50)
        
        # 检查关键参数
        key_params = [
            'robot_description',
            'robot_description_semantic',
            'robot_description_kinematics.uf850.kinematics_solver',
            'robot_description_planning.joint_limits.joint1.max_velocity',
            'planning_pipelines',
            'default_planning_pipeline',
            'capabilities',
            'trajectory_execution.allowed_execution_duration_scaling',
            'moveit_controller_manager'
        ]
        
        # 使用同步参数客户端
        from rclpy.parameter import Parameter
        
        # 等待服务可用
        self.get_logger().info('Waiting for move_group parameter services...')
        
        param_client = self.create_client(GetParameters, '/move_group/get_parameters')
        list_client = self.create_client(ListParameters, '/move_group/list_parameters')
        
        if not param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('move_group parameter service not available!')
            return
            
        # 列出所有参数
        list_request = ListParameters.Request()
        list_future = list_client.call_async(list_request)
        rclpy.spin_until_future_complete(self, list_future)
        
        if list_future.result():
            all_params = list_future.result().result.names
            self.get_logger().info(f'Total parameters in move_group: {len(all_params)}')
            
            # 检查关键参数是否存在
            self.get_logger().info('\nChecking key parameters:')
            for param in key_params:
                if param in all_params:
                    self.get_logger().info(f'✓ {param}')
                else:
                    # 检查是否是前缀匹配
                    matching = [p for p in all_params if p.startswith(param)]
                    if matching:
                        self.get_logger().info(f'✓ {param} (found {len(matching)} matching params)')
                    else:
                        self.get_logger().warn(f'✗ {param} NOT FOUND')
            
            # 获取一些关键参数的值
            self.get_logger().info('\nKey parameter values:')
            
            # 获取capabilities
            caps = [p for p in all_params if 'capabilities' in p]
            if caps:
                get_request = GetParameters.Request()
                get_request.names = caps
                get_future = param_client.call_async(get_request)
                rclpy.spin_until_future_complete(self, get_future)
                
                if get_future.result():
                    for i, param_value in enumerate(get_future.result().values):
                        self.get_logger().info(f'  {caps[i]}: {param_value.string_value}')
                        if 'ExecuteTaskSolutionCapability' in param_value.string_value:
                            self.get_logger().info('  ✓ MTC ExecuteTaskSolutionCapability is loaded!')
            
            # 检查是否有UF850相关的参数
            uf850_params = [p for p in all_params if 'uf850' in p.lower()]
            self.get_logger().info(f'\nFound {len(uf850_params)} UF850-specific parameters')
            
            # 检查是否有gripper参数
            gripper_params = [p for p in all_params if 'gripper' in p.lower()]
            self.get_logger().info(f'Found {len(gripper_params)} gripper-related parameters')
            
        self.get_logger().info('\n' + '=' * 50)
        self.get_logger().info('Configuration check completed!')
        self.get_logger().info('=' * 50)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        checker = MoveItConfigChecker()
        # 运行一次检查后退出
        rclpy.spin_once(checker, timeout_sec=10.0)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 