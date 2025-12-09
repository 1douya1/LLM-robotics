#!/usr/bin/env python3
"""
系统诊断脚本 - 检查MTC任务系统各组件状态
帮助用户了解需要启动哪些服务才能让系统就绪
"""

import rclpy
from rclpy.node import Node
import time
import sys

def check_component_status():
    """检查各组件状态并提供启动建议"""
    
    print("🔍 MTC任务系统诊断中...")
    print("=" * 50)
    
    rclpy.init()
    node = rclpy.create_node('system_diagnosis')
    
    status = {
        "moveit_available": False,
        "execute_pour_available": False,
        "modular_task_available": False,
        "joint_states_available": False,
        "robot_description_available": False
    }
    
    try:
        # 1. 检查MoveIt Move Group Action服务器
        print("📋 检查MoveIt Move Group...")
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup
        
        moveit_client = ActionClient(node, MoveGroup, '/move_action')
        if moveit_client.wait_for_server(timeout_sec=3.0):
            print("✅ MoveIt Move Group: 可用")
            status["moveit_available"] = True
        else:
            print("❌ MoveIt Move Group: 不可用")
        
        # 2. 检查Execute Pour服务器
        print("\n📋 检查Execute Pour Action服务器...")
        try:
            import mtc_interface.action
            pour_client = ActionClient(node, mtc_interface.action.ExecutePour, '/execute_pour')
            if pour_client.wait_for_server(timeout_sec=3.0):
                print("✅ Execute Pour Server: 可用")
                status["execute_pour_available"] = True
            else:
                print("❌ Execute Pour Server: 不可用")
        except ImportError:
            print("❌ Execute Pour Server: mtc_interface未找到")
        
        # 3. 检查模块化任务服务器
        print("\n📋 检查Modular Task服务器...")
        try:
            modular_client = ActionClient(node, mtc_interface.action.ExecutePour, '/execute_modular_task')
            if modular_client.wait_for_server(timeout_sec=3.0):
                print("✅ Modular Task Server: 可用")
                status["modular_task_available"] = True
            else:
                print("❌ Modular Task Server: 不可用")
        except:
            print("❌ Modular Task Server: 不可用")
        
        # 4. 检查关节状态
        print("\n📋 检查关节状态...")
        from sensor_msgs.msg import JointState
        joint_received = False
        
        def joint_callback(msg):
            nonlocal joint_received
            joint_received = True
            print(f"✅ Joint States: 接收到 {len(msg.name)} 个关节状态")
            if 'drive_joint' in msg.name:
                print("  - UF850夹爪关节状态正常")
            status["joint_states_available"] = True
        
        joint_sub = node.create_subscription(JointState, '/joint_states', joint_callback, 1)
        
        # 等待关节状态
        start_time = time.time()
        while not joint_received and (time.time() - start_time) < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if not joint_received:
            print("❌ Joint States: 未接收到关节状态")
        
        # 5. 检查机器人描述
        print("\n📋 检查机器人描述...")
        try:
            robot_desc = node.get_parameter_or('robot_description', None)
            if robot_desc is not None and robot_desc.value:
                print("✅ Robot Description: 可用")
                status["robot_description_available"] = True
            else:
                print("❌ Robot Description: 不可用")
        except:
            print("❌ Robot Description: 获取失败")
        
        # 6. 检查话题列表
        print("\n📋 检查重要话题...")
        topic_list = node.get_topic_names_and_types()
        important_topics = ['/joint_states', '/robot_description', '/move_group/display_planned_path']
        
        for topic_name, _ in topic_list:
            if any(important in topic_name for important in important_topics):
                print(f"✅ 话题: {topic_name}")
        
    except Exception as e:
        print(f"❌ 诊断过程中出现错误: {e}")
    
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    # 总结和建议
    print("\n" + "=" * 50)
    print("📊 诊断结果总结:")
    
    ready_count = sum(status.values())
    total_count = len(status)
    
    print(f"✅ 可用组件: {ready_count}/{total_count}")
    
    if ready_count == total_count:
        print("🎉 系统完全就绪！可以执行MTC任务。")
    else:
        print("⚠️ 系统未完全就绪，需要启动以下组件：")
        
        if not status["moveit_available"]:
            print("\n🚀 启动MoveIt:")
            print("   ros2 launch xarm_moveit_config demo.launch.py")
        
        if not status["execute_pour_available"]:
            print("\n🚀 启动Execute Pour服务器:")
            print("   ros2 run mtc_tutorial execute_pour_server")
        
        if not status["modular_task_available"]:
            print("\n🚀 启动模块化任务服务器:")
            print("   ros2 run mtc_tutorial modular_task_server")
        
        if not status["joint_states_available"]:
            print("\n🚀 启动关节状态发布:")
            print("   # 如果是仿真环境:")
            print("   ros2 run joint_state_publisher_gui joint_state_publisher_gui")
            print("   # 如果是真实机器人，启动机器人驱动")
        
        if not status["robot_description_available"]:
            print("\n🚀 加载机器人描述:")
            print("   ros2 launch xarm_description robot_state_publisher.launch.py")
    
    return status

def quick_start_guide():
    """快速启动指南"""
    print("\n" + "=" * 50)
    print("🚀 快速启动指南")
    print("=" * 50)
    
    print("📋 完整启动序列（建议按顺序执行）：")
    print()
    print("1️⃣ 启动机器人描述和状态发布:")
    print("   ros2 launch xarm_description robot_state_publisher.launch.py")
    print()
    print("2️⃣ 启动MoveIt规划环境:")
    print("   ros2 launch xarm_moveit_config demo.launch.py")
    print()
    print("3️⃣ 启动关节状态发布（仿真）:")
    print("   ros2 run joint_state_publisher_gui joint_state_publisher_gui")
    print()
    print("4️⃣ 启动MTC任务服务器:")
    print("   ros2 run mtc_tutorial execute_pour_server")
    print("   # 或者启动模块化服务器:")
    print("   ros2 run mtc_tutorial modular_task_server")
    print()
    print("5️⃣ 验证系统状态:")
    print("   python3 src/mtc_tutorial/scripts/system_diagnosis.py")
    print()
    print("📝 注意事项:")
    print("   - 每个命令在单独的终端窗口中运行")
    print("   - 等待每个组件完全启动后再启动下一个")
    print("   - 如果使用真实机器人，替换关节状态发布为机器人驱动")

def main():
    """主函数"""
    print("🔧 MTC任务系统诊断工具")
    
    if len(sys.argv) > 1 and sys.argv[1] == "--guide":
        quick_start_guide()
        return
    
    try:
        status = check_component_status()
        
        # 如果系统未就绪，显示快速启动指南
        if not all(status.values()):
            print("\n💡 使用 --guide 参数查看详细启动指南:")
            print("   python3 src/mtc_tutorial/scripts/system_diagnosis.py --guide")
        
    except KeyboardInterrupt:
        print("\n⏹️ 诊断已中断")
    except Exception as e:
        print(f"\n❌ 诊断失败: {e}")

if __name__ == "__main__":
    main() 