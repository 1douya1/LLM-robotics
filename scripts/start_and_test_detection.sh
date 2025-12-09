#!/bin/bash
# One-Shot 物体检测启动和测试脚本

echo "=== One-Shot物体检测系统启动脚本 ==="
echo

# 设置工作空间
cd /home/wenhao/uf_custom_ws
source install/setup.bash

# 检查必要的依赖
echo "1. 检查依赖..."

# 检查YOLO
if python3 -c "from ultralytics import YOLO; print('YOLO OK')" 2>/dev/null; then
    echo "✅ YOLO 已安装"
else
    echo "❌ YOLO 未安装，正在安装..."
    pip install ultralytics
fi

# 检查RealSense
if ros2 pkg list | grep -q realsense2_camera; then
    echo "✅ RealSense 驱动已安装"
else
    echo "❌ RealSense 驱动未安装"
    echo "请运行: sudo apt install ros-humble-realsense2-camera"
    exit 1
fi

echo

# 选择启动模式
echo "2. 选择启动模式:"
echo "  [1] 完整模式 (相机 + 手眼标定 + 检测)"
echo "  [2] 仅检测模式 (不启动相机和手眼标定) - 推荐"
echo "  [3] 纯检测启动 (使用简化启动文件)"
echo "  [4] 测试模式 (仅测试已运行的检测节点)"
echo
read -p "请选择模式 [1-4]: " mode

case $mode in
    1)
        echo "启动完整模式..."
        echo "请确保:"
        echo "- RealSense相机已连接"
        echo "- 手眼标定已完成"
        echo
        read -p "按Enter继续..."
        
        echo "启动系统..."
        ros2 launch mtc_tutorial oneshot_object_detection.launch.py &
        LAUNCH_PID=$!
        
        echo "等待系统初始化..."
        sleep 10
        
        echo "触发检测测试..."
        ros2 service call /trigger_object_detection std_srvs/srv/Trigger
        
        echo "系统正在运行，按Ctrl+C退出"
        wait $LAUNCH_PID
        ;;
        
    2)
        echo "启动仅检测模式..."
        echo "注意: 相机和手眼标定需要单独启动"
        echo
        
        ros2 launch mtc_tutorial oneshot_object_detection.launch.py \
            launch_camera:=false \
            launch_handeye_tf:=false &
        LAUNCH_PID=$!
        
        echo "等待检测节点初始化..."
        sleep 5
        
        echo "检测节点已启动，可以通过以下命令触发检测:"
        echo "  ros2 service call /trigger_object_detection std_srvs/srv/Trigger"
        echo
        echo "按Ctrl+C退出"
        wait $LAUNCH_PID
        ;;
        
    3)
        echo "启动纯检测模式..."
        echo "使用简化启动文件，仅启动检测和标记发布器"
        echo
        
        ros2 launch mtc_tutorial detection_only.launch.py &
        LAUNCH_PID=$!
        
        echo "等待检测节点初始化..."
        sleep 5
        
        echo "检测节点已启动，可以通过以下命令触发检测:"
        echo "  ros2 service call /trigger_object_detection std_srvs/srv/Trigger"
        echo
        echo "按Ctrl+C退出"
        wait $LAUNCH_PID
        ;;
        
    4)
        echo "运行检测测试..."
        python3 test_oneshot_detection.py
        ;;
        
    *)
        echo "无效选择"
        exit 1
        ;;
esac 