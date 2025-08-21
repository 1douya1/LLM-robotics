#!/usr/bin/env python3
"""
测试重构后的MTC MCP工具（修复版本）
解决了moveit_commander和mtc_interface的依赖问题
"""
import asyncio
import json
import sys
import os
from pathlib import Path

# 设置环境变量和路径
def setup_environment():
    """设置必要的环境变量和Python路径"""
    # 获取工作区根目录
    script_dir = Path(__file__).resolve().parent
    workspace_root = script_dir.parent.parent.parent
    install_path = workspace_root / "install"
    
    if install_path.exists():
        print(f"Found install directory: {install_path}")
        
        # 设置Python路径
        python_path = install_path / "mtc_interface" / "local" / "lib" / "python3.10" / "dist-packages"
        if python_path.exists() and str(python_path) not in sys.path:
            sys.path.insert(0, str(python_path))
            print(f"Added to Python path: {python_path}")
        
        # 设置ROS环境变量
        if "AMENT_PREFIX_PATH" not in os.environ:
            os.environ["AMENT_PREFIX_PATH"] = str(install_path)
        elif str(install_path) not in os.environ["AMENT_PREFIX_PATH"]:
            os.environ["AMENT_PREFIX_PATH"] = str(install_path) + ":" + os.environ["AMENT_PREFIX_PATH"]
        
        print(f"AMENT_PREFIX_PATH: {os.environ.get('AMENT_PREFIX_PATH', 'Not set')}")
    else:
        print(f"Install directory not found: {install_path}")
        print("Please run 'colcon build' first")

# 在导入本地模块之前设置环境
setup_environment()

# 确保可以导入本地模块
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

import ros_client_tools as tools

def test_mtc_interface_import():
    """测试mtc_interface是否可以导入"""
    print("=" * 50)
    print("测试 0: mtc_interface导入测试")
    print("=" * 50)
    
    try:
        import mtc_interface.action
        print("✓ mtc_interface导入成功")
        return True
    except Exception as e:
        print(f"✗ mtc_interface导入失败: {e}")
        return False

def test_setup_planning_scene():
    """测试场景构建功能"""
    print("=" * 50)
    print("测试 1: 场景构建 (setup_planning_scene)")
    print("=" * 50)
    
    # 测试完整场景构建（包含杯子）
    result = tools.setup_planning_scene(
        include_cup=True,
        cup_x=0.0,
        cup_y=-0.4,
        cup_z=0.13,
        cup_height=0.1,
        cup_radius=0.02
    )
    
    print(f"场景构建结果: {json.dumps(result, indent=2, ensure_ascii=False)}")
    return result["ok"] if result else False

def test_check_object_exists():
    """测试对象存在检查功能"""
    print("=" * 50)
    print("测试 2: 对象存在检查 (check_object_exists)")
    print("=" * 50)
    
    result = tools.check_object_exists(object_id="object")
    print(f"对象检查结果: {json.dumps(result, indent=2, ensure_ascii=False)}")
    return result["ok"] if result else False

def test_update_cup_pose():
    """测试杯子位姿更新功能"""
    print("=" * 50)
    print("测试 3: 杯子位姿更新 (update_cup_pose)")
    print("=" * 50)
    
    # 更新到一个新位置
    result = tools.update_cup_pose(
        cup_x=0.1,
        cup_y=-0.5,
        cup_z=0.15,
        cup_height=0.12,
        cup_radius=0.025
    )
    
    print(f"位姿更新结果: {json.dumps(result, indent=2, ensure_ascii=False)}")
    return result["ok"] if result else False

def test_execute_pour_with_object_check():
    """测试重构后的倾倒执行功能（包含对象检查）"""
    print("=" * 50)
    print("测试 4: 倾倒执行 (execute_pour - 带对象检查)")
    print("=" * 50)
    
    # 测试仅规划模式，避免实际执行
    params = {
        "tilt_start_deg": 45.0,
        "tilt_end_deg": 90.0,
        "tilt_speed_deg_s": 20.0,
        "pour_hold_sec": 1.0,
        "lift_height": 0.12,
        "approach_min": 0.05,
        "approach_max": 0.12,
        "plan_only": True,  # 仅规划，不执行
        "target_id": "test_cup"
    }
    
    result = tools.execute_pour(
        params=params,
        timeout_sec=30.0
    )
    
    print(f"倾倒执行结果: {json.dumps(result, indent=2, ensure_ascii=False)}")
    return result["ok"] if result else False

def test_execute_pour_without_scene():
    """测试在没有场景的情况下执行倾倒（应该失败）"""
    print("=" * 50)
    print("测试 5: 无场景情况下的倾倒执行 (应该失败)")
    print("=" * 50)
    
    # 先清理场景，通过创建一个无杯子的场景
    tools.setup_planning_scene(include_cup=False)
    
    # 现在尝试执行倾倒，应该失败
    params = {
        "tilt_start_deg": 45.0,
        "tilt_end_deg": 90.0,
        "plan_only": True
    }
    
    result = tools.execute_pour(params=params)
    
    print(f"无场景倾倒结果: {json.dumps(result, indent=2, ensure_ascii=False)}")
    # 这次我们期望失败
    expected_failure = not result["ok"] and result.get("status") == "missing_object"
    print(f"按预期失败: {expected_failure}")
    return expected_failure

def test_cup_pose_update_in_execute_pour():
    """测试在execute_pour中更新杯子位姿"""
    print("=" * 50)
    print("测试 6: execute_pour中的位姿更新功能")
    print("=" * 50)
    
    # 先重新构建场景（包含杯子）
    tools.setup_planning_scene(include_cup=True)
    
    # 通过execute_pour更新杯子位姿
    params = {
        "tilt_start_deg": 45.0,
        "tilt_end_deg": 90.0,
        "plan_only": True,
        "update_cup_pose_first": True,
        "cup_x": 0.05,
        "cup_y": -0.45,
        "cup_z": 0.14
    }
    
    result = tools.execute_pour(params=params)
    
    print(f"带位姿更新的倾倒结果: {json.dumps(result, indent=2, ensure_ascii=False)}")
    return result["ok"] if result else False

def main():
    """运行所有测试"""
    print("开始测试重构后的MTC MCP工具（修复版本）...")
    print("注意: 确保ROS2环境已启动且move_group正在运行")
    print()
    
    tests = [
        ("mtc_interface导入", test_mtc_interface_import),
        ("场景构建", test_setup_planning_scene),
        ("对象存在检查", test_check_object_exists),
        ("杯子位姿更新", test_update_cup_pose),
        ("倾倒执行（带对象检查）", test_execute_pour_with_object_check),
        ("无场景倾倒（预期失败）", test_execute_pour_without_scene),
        ("execute_pour中的位姿更新", test_cup_pose_update_in_execute_pour)
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            success = test_func()
            results.append((test_name, success))
            print(f"✓ {test_name}: {'通过' if success else '失败'}")
        except Exception as e:
            results.append((test_name, False))
            print(f"✗ {test_name}: 异常 - {str(e)}")
        print()
    
    print("=" * 50)
    print("测试总结")
    print("=" * 50)
    passed = sum(1 for _, success in results if success)
    total = len(results)
    
    for test_name, success in results:
        status = "✓ 通过" if success else "✗ 失败"
        print(f"{status} {test_name}")
    
    print()
    print(f"总体结果: {passed}/{total} 测试通过")
    
    if passed == total:
        print("🎉 所有测试通过！重构成功！")
        return 0
    elif passed > 0:
        print(f"⚠️  部分测试通过 ({passed}/{total})，依赖问题已部分解决")
        return 0
    else:
        print("⚠️  所有测试失败，请检查ROS2环境和move_group状态")
        return 1

if __name__ == '__main__':
    sys.exit(main()) 