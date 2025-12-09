#!/bin/bash

# UF850 手眼标定系统启动脚本
# 作者: 自动生成于标定完成后
# 日期: $(date)

echo "🚀 启动UF850手眼标定系统..."

# 设置工作空间
cd /home/wenhao/uf_custom_ws
source install/setup.bash

echo "✅ 启动以下终端窗口："
echo "1. RealSense相机"
echo "2. UF850机械臂"  
echo "3. 手眼标定发布"
echo "4. TF连接"

# 提示用户在不同终端运行以下命令
echo ""
echo "📋 请在不同终端中依次运行："
echo ""
echo "# 终端1: 相机"
echo "cd /home/wenhao/uf_custom_ws && source install/setup.bash"
echo "ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true"
echo ""
echo "# 终端2: 机械臂"
echo "cd /home/wenhao/uf_custom_ws && source install/setup.bash"
echo "export QT_ENABLE_HIGHDPI_SCALING=0" # 解决QT高分辨率缩放问题
echo "ros2 launch mtc_tutorial pour_demo.launch.py"
echo ""
echo "# 终端3: 标定发布"
echo "cd /home/wenhao/uf_custom_ws && source install/setup.bash"
echo "ros2 launch src/charuco_handeye_publish.launch.py"
echo ""
echo "# 终端4: TF连接 (关键!)"
echo "cd /home/wenhao/uf_custom_ws && source install/setup.bash"
echo "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world camera_link"
echo ""
echo "🎯 验证命令："
echo "ros2 run tf2_ros tf2_echo link_base camera_color_optical_frame"
echo ""
echo "🎉 您的标定数值: Translation[0.457, -0.473, 0.321] 非常优秀旧版!" 
echo "用于查看TF树" 
echo "ros2 run tf2_tools view_frames"
echo "ros2 run tf2_ros tf2_echo link_base camera_color_optical_frame"
