#!/usr/bin/env python3
"""
MCP问题诊断和修复脚本
帮助识别和解决测试中遇到的各种问题
"""

import os
import sys
import subprocess
import traceback
from pathlib import Path
from typing import Dict, List, Any

def check_ros2_environment() -> Dict[str, Any]:
    """检查ROS2环境设置"""
    print("🔍 检查ROS2环境...")
    
    result = {
        "domain_id": os.environ.get("ROS_DOMAIN_ID", "未设置"),
        "distro": os.environ.get("ROS_DISTRO", "未设置"),  
        "ament_prefix_path": bool(os.environ.get("AMENT_PREFIX_PATH")),
        "python_path_count": len(sys.path),
        "workspace_sourced": False
    }
    
    # 检查工作空间是否正确sourced
    ament_prefix = os.environ.get("AMENT_PREFIX_PATH", "")
    workspace_install = str(Path(__file__).parent.parent.parent.parent / "install")
    if workspace_install in ament_prefix:
        result["workspace_sourced"] = True
    
    return result

def check_mtc_interface_import() -> Dict[str, Any]:
    """检查mtc_interface包的导入"""
    print("🔍 检查mtc_interface包导入...")
    
    result = {
        "can_import_basic": False,
        "can_import_action": False,
        "can_import_execute_pour": False,
        "python_path_has_install": False,
        "error_details": []
    }
    
    # 检查Python路径中是否包含install目录
    workspace_root = Path(__file__).parent.parent.parent.parent
    install_python_path = workspace_root / "install" / "mtc_interface" / "local" / "lib" / "python3.10" / "dist-packages"
    
    if str(install_python_path) in sys.path:
        result["python_path_has_install"] = True
    else:
        # 动态添加路径
        if install_python_path.exists():
            sys.path.insert(0, str(install_python_path))
            result["python_path_has_install"] = True
            print(f"✅ 动态添加Python路径: {install_python_path}")
    
    # 测试基本导入
    try:
        import mtc_interface
        result["can_import_basic"] = True
        print("✅ 成功导入mtc_interface基础包")
    except Exception as e:
        result["error_details"].append(f"基础导入失败: {str(e)}")
        print(f"❌ 基础导入失败: {str(e)}")
    
    # 测试action导入
    try:
        import mtc_interface.action
        result["can_import_action"] = True
        print("✅ 成功导入mtc_interface.action")
    except Exception as e:
        result["error_details"].append(f"action导入失败: {str(e)}")
        print(f"❌ action导入失败: {str(e)}")
    
    # 测试ExecutePour导入
    try:
        import mtc_interface.action
        action = mtc_interface.action.ExecutePour
        result["can_import_execute_pour"] = True
        print("✅ 成功导入ExecutePour Action")
    except Exception as e:
        result["error_details"].append(f"ExecutePour导入失败: {str(e)}")
        print(f"❌ ExecutePour导入失败: {str(e)}")
    
    return result

def check_ros2_services_and_actions() -> Dict[str, Any]:
    """检查ROS2服务和Action的可用性"""
    print("🔍 检查ROS2服务和Action...")
    
    result = {
        "modular_task_server_running": False,
        "parameter_service_available": False,
        "action_server_available": False,
        "move_group_running": False,
        "action_server_details": {}
    }
    
    try:
        # 检查节点
        nodes_result = subprocess.run(["ros2", "node", "list"], 
                                    capture_output=True, text=True, timeout=10)
        if nodes_result.returncode == 0:
            nodes = nodes_result.stdout.strip().split('\n')
            result["modular_task_server_running"] = "/modular_task_server" in nodes
            result["move_group_running"] = "/move_group" in nodes
            
            print(f"✅ 模块化任务服务器运行: {result['modular_task_server_running']}")
            print(f"✅ MoveGroup运行: {result['move_group_running']}")
    except Exception as e:
        print(f"❌ 检查节点失败: {str(e)}")
    
    try:
        # 检查参数服务
        param_result = subprocess.run(["ros2", "service", "list"], 
                                    capture_output=True, text=True, timeout=10)
        if param_result.returncode == 0:
            services = param_result.stdout
            result["parameter_service_available"] = "/modular_task_server/set_parameters" in services
            print(f"✅ 参数服务可用: {result['parameter_service_available']}")
    except Exception as e:
        print(f"❌ 检查参数服务失败: {str(e)}")
    
    try:
        # 检查Action服务器
        action_result = subprocess.run(["ros2", "action", "list"], 
                                     capture_output=True, text=True, timeout=10)
        if action_result.returncode == 0:
            actions = action_result.stdout
            result["action_server_available"] = "/execute_modular_task" in actions
            print(f"✅ Action服务器可用: {result['action_server_available']}")
            
            # 获取Action详细信息
            if result["action_server_available"]:
                try:
                    info_result = subprocess.run(["ros2", "action", "info", "/execute_modular_task"], 
                                               capture_output=True, text=True, timeout=5)
                    if info_result.returncode == 0:
                        result["action_server_details"]["info"] = info_result.stdout
                except Exception:
                    pass
                    
    except Exception as e:
        print(f"❌ 检查Action服务器失败: {str(e)}")
    
    return result

def test_parameter_setting() -> Dict[str, Any]:
    """测试参数设置功能"""
    print("🔍 测试参数设置...")
    
    result = {
        "can_set_test_param": False,
        "error_message": None
    }
    
    try:
        # 尝试设置一个测试参数
        param_result = subprocess.run([
            "ros2", "param", "set", "/modular_task_server", "test_param", "123"
        ], capture_output=True, text=True, timeout=10)
        
        if param_result.returncode == 0:
            result["can_set_test_param"] = True
            print("✅ 参数设置测试成功")
            
            # 清理测试参数
            try:
                subprocess.run([
                    "ros2", "service", "call", "/modular_task_server/set_parameters", 
                    "rcl_interfaces/srv/SetParameters", 
                    "parameters: [{name: test_param, value: {type: 0}}]"
                ], capture_output=True, timeout=5)
            except:
                pass
        else:
            result["error_message"] = param_result.stderr
            print(f"❌ 参数设置失败: {param_result.stderr}")
            
    except Exception as e:
        result["error_message"] = str(e)
        print(f"❌ 参数设置测试异常: {str(e)}")
    
    return result

def test_simple_ros_import() -> Dict[str, Any]:
    """测试基础ROS2导入"""
    print("🔍 测试基础ROS2导入...")
    
    result = {
        "rclpy": False,
        "action_client": False,
        "basic_msgs": False,
        "error_details": []
    }
    
    try:
        import rclpy
        result["rclpy"] = True
        print("✅ rclpy导入成功")
    except Exception as e:
        result["error_details"].append(f"rclpy: {str(e)}")
        print(f"❌ rclpy导入失败: {str(e)}")
    
    try:
        from rclpy.action import ActionClient
        result["action_client"] = True
        print("✅ ActionClient导入成功")
    except Exception as e:
        result["error_details"].append(f"ActionClient: {str(e)}")
        print(f"❌ ActionClient导入失败: {str(e)}")
    
    try:
        import std_msgs.msg
        result["basic_msgs"] = True
        print("✅ 基础消息类型导入成功")
    except Exception as e:
        result["error_details"].append(f"std_msgs: {str(e)}")
        print(f"❌ 基础消息类型导入失败: {str(e)}")
    
    return result

def suggest_fixes(diagnosis_results: Dict[str, Any]) -> List[str]:
    """根据诊断结果建议修复方案"""
    suggestions = []
    
    # 检查ROS2环境
    env_result = diagnosis_results.get("environment", {})
    if not env_result.get("workspace_sourced", False):
        suggestions.append("🔧 工作空间未正确sourced，运行: source install/setup.bash")
    
    if env_result.get("distro") == "未设置":
        suggestions.append("🔧 ROS_DISTRO未设置，运行: source /opt/ros/humble/setup.bash")
    
    # 检查mtc_interface导入
    import_result = diagnosis_results.get("mtc_interface", {})
    if not import_result.get("can_import_basic", False):
        suggestions.append("🔧 mtc_interface基础包无法导入，重新编译: colcon build --packages-select mtc_interface")
    
    if not import_result.get("python_path_has_install", False):
        suggestions.append("🔧 Python路径缺少install目录，添加到PYTHONPATH或重新source工作空间")
    
    # 检查服务状态
    services_result = diagnosis_results.get("services", {})
    if not services_result.get("modular_task_server_running", False):
        suggestions.append("🔧 模块化任务服务器未运行，启动: ros2 run mtc_tutorial modular_task_server")
    
    if not services_result.get("move_group_running", False):
        suggestions.append("🔧 MoveGroup未运行，启动完整的机器人系统")
    
    # 检查参数设置
    param_result = diagnosis_results.get("parameter_test", {})
    if not param_result.get("can_set_test_param", False):
        suggestions.append("🔧 参数设置失败，检查modular_task_server是否正确初始化参数服务")
    
    return suggestions

def main():
    """主诊断程序"""
    print("🩺 MCP问题诊断器")
    print("=" * 60)
    
    # 执行各项检查
    diagnosis_results = {}
    
    print("\n📋 执行诊断检查...")
    
    try:
        diagnosis_results["environment"] = check_ros2_environment()
        diagnosis_results["basic_ros"] = test_simple_ros_import()
        diagnosis_results["mtc_interface"] = check_mtc_interface_import()
        diagnosis_results["services"] = check_ros2_services_and_actions()
        diagnosis_results["parameter_test"] = test_parameter_setting()
        
    except KeyboardInterrupt:
        print("\n⏹️  诊断被用户中断")
        return
    except Exception as e:
        print(f"\n❌ 诊断过程中发生错误: {str(e)}")
        traceback.print_exc()
        return
    
    # 生成诊断报告
    print("\n📊 诊断报告")
    print("-" * 40)
    
    # 总结关键状态
    env_ok = diagnosis_results["environment"].get("workspace_sourced", False)
    import_ok = diagnosis_results["mtc_interface"].get("can_import_execute_pour", False)
    services_ok = (diagnosis_results["services"].get("modular_task_server_running", False) and
                   diagnosis_results["services"].get("parameter_service_available", False))
    
    print(f"🏠 ROS2环境: {'✅ 正常' if env_ok else '❌ 有问题'}")
    print(f"📦 mtc_interface导入: {'✅ 正常' if import_ok else '❌ 有问题'}")
    print(f"🔧 ROS2服务: {'✅ 正常' if services_ok else '❌ 有问题'}")
    
    # 建议修复方案
    suggestions = suggest_fixes(diagnosis_results)
    
    if suggestions:
        print(f"\n💡 修复建议:")
        for i, suggestion in enumerate(suggestions, 1):
            print(f"  {i}. {suggestion}")
    else:
        print(f"\n🎉 所有检查都通过了！如果测试仍然失败，可能是临时的网络或系统问题。")
    
    # 提供快速修复命令
    print(f"\n⚡ 快速修复命令:")
    print("# 重新source工作空间")
    print("source install/setup.bash")
    print()
    print("# 重新编译mtc_interface (如果导入有问题)")
    print("colcon build --packages-select mtc_interface --cmake-clean-cache")
    print()
    print("# 重新编译整个项目 (如果有严重问题)")  
    print("colcon build --cmake-clean-cache")
    print()
    print("# 检查系统状态")
    print("ros2 node list | grep modular_task_server")
    print("ros2 action info /execute_modular_task")

if __name__ == "__main__":
    main() 