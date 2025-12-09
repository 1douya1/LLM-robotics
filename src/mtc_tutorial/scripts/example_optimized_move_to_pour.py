#!/usr/bin/env python3
"""
优化后的move_to_pour_position功能使用示例
展示高级Cartesian规划和姿态稳定控制
"""

import time
import ros_client_tools as tools

def example_basic_usage():
    """基本使用示例"""
    print("🔧 基本使用示例：移动到绝对位置")
    
    result = tools.move_to_pour_position(
        target_position={"x": 0.2, "y": -0.6, "z": 0.25},
        movement_mode="absolute"
    )
    
    print(f"结果: {result.get('success', False)}")
    if result.get('features_used'):
        print("使用的功能:")
        for feature in result['features_used']:
            print(f"  - {feature}")
    
    return result.get('success', False)


def example_advanced_cartesian():
    """高级Cartesian优化示例"""
    print("\n🎯 高级配置示例：最大化Cartesian路径稳定性")
    
    result = tools.move_to_pour_position(
        target_position={"x": 0.15, "y": -0.7, "z": 0.3},
        movement_mode="absolute",
        motion_params={
            "velocity_scaling": 0.1,          # 更慢更稳定
            "min_cartesian_fraction": 0.95,   # 要求95%笛卡尔路径
            "maintain_current_orientation": True,  # 保持当前姿态
            "planning_attempts": 5,           # 更多规划尝试
            "enable_collision_checking": True,
            "step_size": 0.003               # 更精细的步长
        }
    )
    
    print(f"高级Cartesian结果: {result.get('success', False)}")
    if 'advanced_config' in result:
        config = result['advanced_config']
        print(f"笛卡尔优化: {config['cartesian_optimization']}")
        print(f"姿态控制: {config['orientation_control']}")
    
    return result.get('success', False)


def example_relative_movement():
    """相对移动示例"""
    print("\n🔄 相对移动示例：精细位置调整")
    
    result = tools.move_to_pour_position(
        target_position={"x": 0.0, "y": 0.0, "z": 0.0},  # 占位符
        movement_mode="relative",
        relative_movement={"x": 0.0, "y": -0.15, "z": 0.05},
        motion_params={
            "velocity_scaling": 0.08,         # 非常慢的相对移动
            "min_cartesian_fraction": 0.9,   # 高笛卡尔要求
            "maintain_current_orientation": True
        }
    )
    
    print(f"相对移动结果: {result.get('success', False)}")
    return result.get('success', False)


def example_custom_orientation():
    """自定义姿态示例"""
    print("\n🎪 自定义姿态示例：指定倾倒角度")
    
    result = tools.move_to_pour_position(
        target_position={"x": 0.1, "y": -0.5, "z": 0.2},
        movement_mode="absolute",
        motion_params={
            "maintain_current_orientation": False,  # 使用指定姿态
            "target_roll": 0.0,
            "target_pitch": 0.2,                   # 向前倾斜更多
            "target_yaw": 0.0,
            "velocity_scaling": 0.12,
            "min_cartesian_fraction": 0.85
        }
    )
    
    print(f"自定义姿态结果: {result.get('success', False)}")
    return result.get('success', False)


def example_long_distance_segmented():
    """长距离分段移动示例"""
    print("\n📏 长距离移动示例：自动分段策略")
    
    # 模拟一个较长距离的移动（>20cm触发分段）
    result = tools.move_to_pour_position(
        target_position={"x": 0.0, "y": 0.0, "z": 0.0},  # 占位符
        movement_mode="relative", 
        relative_movement={"x": 0.1, "y": -0.25, "z": 0.1},  # 总距离约28cm
        motion_params={
            "velocity_scaling": 0.15,
            "min_cartesian_fraction": 0.8,
            "maintain_current_orientation": True
        }
    )
    
    print(f"长距离分段移动结果: {result.get('success', False)}")
    if result.get('note'):
        print(f"优化说明: {result['note']}")
    
    return result.get('success', False)


def main():
    """主函数：运行所有示例"""
    print("🚀 优化后的move_to_pour_position功能演示")
    print("=" * 50)
    
    # 检查系统状态
    state = tools.get_task_state()
    print(f"系统状态: {state.get('stage', 'unknown')}")
    print(f"系统就绪: {state.get('system_ready', False)}")
    
    if not state.get('system_ready', False):
        print("⚠️ 系统未就绪，请先启动MoveIt和机器人")
        return
    
    # 运行示例（仅规划，不实际执行）
    examples = [
        ("基本使用", example_basic_usage),
        ("高级Cartesian", example_advanced_cartesian), 
        ("相对移动", example_relative_movement),
        ("自定义姿态", example_custom_orientation),
        ("长距离分段", example_long_distance_segmented)
    ]
    
    results = []
    for name, func in examples:
        try:
            success = func()
            results.append((name, success))
            time.sleep(1)  # 短暂等待
        except Exception as e:
            print(f"❌ {name}示例执行失败: {e}")
            results.append((name, False))
    
    # 总结
    print("\n" + "=" * 50)
    print("📊 示例执行结果总结:")
    for name, success in results:
        status = "✅ 成功" if success else "❌ 失败"
        print(f"  {name}: {status}")
    
    success_rate = sum(1 for _, success in results if success) / len(results)
    print(f"\n🎯 成功率: {success_rate:.1%}")
    
    print("\n🔧 优化特性说明:")
    print("  - 智能规划器选择（Cartesian优先+Pipeline备用）")
    print("  - 自动姿态管理（保持当前或指定目标）")
    print("  - 长距离自动分段移动")
    print("  - 高度可配置的Cartesian参数")
    print("  - 增强的错误处理和容错性")


if __name__ == "__main__":
    main() 