#!/bin/bash

# MTC Tutorial 安全启动脚本
# 自动清理之前的节点并启动新的演示

echo "🔧 准备启动 MTC Tutorial 演示..."

# 1. 清理可能存在的旧节点
echo "🧹 清理旧的 MTC 节点..."
pkill -f mtc_tutorial 2>/dev/null || true
sleep 1

# 2. 检查 move_group 是否运行
if ! pgrep -f "move_group" > /dev/null; then
    echo "❌ move_group 未运行！"
    echo "请先启动机器人仿真环境（MTC专用版本）："
    echo "   ros2 launch xarm_moveit_config uf850_moveit_gazebo_mtc.launch.py"
    exit 1
fi

# 3. 检查控制器状态
echo "🔍 检查控制器状态..."
if ! ros2 control list_controllers | grep -q "active"; then
    echo "⚠️  控制器可能未正常运行，但继续启动..."
fi

# 4. source 环境并启动
echo "🚀 启动 MTC Tutorial..."
source install/setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py

echo "✅ 启动脚本执行完成" 