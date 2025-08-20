#!/usr/bin/env python3
"""
测试Agent友好的辅助工具
"""
import sys
from pathlib import Path
import json

# 确保可以导入工具模块
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

import ros_client_tools as tools

def test_task_state():
    """测试任务状态获取"""
    print("=" * 50)
    print("🔍 测试任务状态获取工具")
    print("=" * 50)
    
    try:
        state = tools.get_task_state(timeout_sec=3.0)
        print(json.dumps(state, indent=2, ensure_ascii=False))
        
        # 分析状态
        print("\n📊 状态分析:")
        print(f"  - 系统准备状态: {'✅ 就绪' if state.get('system_ready') else '❌ 未就绪'}")
        print(f"  - 当前阶段: {state.get('stage', 'unknown')}")
        print(f"  - 夹爪状态: {state.get('gripper_state', 'unknown')}")
        print(f"  - Action服务器: {state.get('action_status', 'unknown')}")
        
        if state.get('last_error'):
            print(f"  - ⚠️ 最近错误: {state['last_error']}")
            
        return state.get('system_ready', False)
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False

def test_abort_dry_run():
    """测试中止功能 (干运行模式，不真正执行危险操作)"""
    print("\n" + "=" * 50)
    print("🛑 测试中止和重置工具 (干运行)")
    print("=" * 50)
    
    try:
        # 这里我们可以模拟一个中止请求
        result = tools.abort_and_reset(
            reason="测试中止功能",
            timeout_sec=10.0
        )
        print(json.dumps(result, indent=2, ensure_ascii=False))
        
        # 分析结果
        print("\n📊 中止结果分析:")
        print(f"  - 中止成功: {'✅ 是' if result.get('success') else '❌ 否'}")
        print(f"  - 机器人安全: {'✅ 是' if result.get('robot_safe') else '❌ 否'}")
        print(f"  - 中止原因: {result.get('reason', 'unknown')}")
        
        print("\n🔧 执行的动作:")
        for i, action in enumerate(result.get('actions_taken', []), 1):
            print(f"  {i}. {action}")
        
        if result.get('warnings'):
            print("\n⚠️ 警告信息:")
            for i, warning in enumerate(result['warnings'], 1):
                print(f"  {i}. {warning}")
                
        return result.get('success', False)
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("🤖 Agent友好工具测试开始")
    print(f"📁 工作目录: {Path.cwd()}")
    
    # 测试1: 任务状态获取
    system_ready = test_task_state()
    
    # 测试2: 中止功能
    abort_success = test_abort_dry_run()
    
    # 总结
    print("\n" + "=" * 50)
    print("📋 测试总结")
    print("=" * 50)
    print(f"任务状态工具: {'✅ 通过' if system_ready else '⚠️ 系统未就绪'}")
    print(f"中止重置工具: {'✅ 通过' if abort_success else '⚠️ 需要检查'}")
    
    if system_ready:
        print("\n💡 建议:")
        print("  - 系统已就绪，可以执行倾倒任务")
        print("  - 使用 get_task_state() 监控任务进展") 
        print("  - 如有异常，使用 abort_and_reset() 紧急停止")
    else:
        print("\n💡 建议:")
        print("  - 请先启动 execute_pour_server")
        print("  - 确保 MoveIt 和机器人系统正常运行")
        print("  - 检查 ROS2 节点和话题状态")

if __name__ == '__main__':
    main() 