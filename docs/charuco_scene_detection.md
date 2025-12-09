# ChArUco场景检测和可视化指南

## 🎯 功能概述

这个系统可以实时检测ChArUco board在场景中的位置，并在rviz中进行可视化。ChArUco board被检测为一个物体，其位置和方向会以`charuco_board` frame的形式发布到ROS的TF系统中。

## 🚀 快速启动

### 方法1: 使用一键启动脚本（推荐）

```bash
cd /home/wenhao/uf_custom_ws
./start_charuco_scene_detection.sh
```

### 方法2: 手动启动各组件

```bash
# Terminal 1: 启动ChArUco检测器
cd /home/wenhao/uf_custom_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch charuco_handeye_publish.launch.py enable_charuco_publisher:=true

# Terminal 2: 启动rviz
rviz2 -d /home/wenhao/uf_custom_ws/charuco_visualization.rviz
```

## 📋 使用步骤

1. **📐 准备ChArUco Board**
   - 使用项目中的`board.jpg`打印ChArUco board
   - 确保打印尺寸正确（square_length: 25mm, marker_length: 18mm）
   - 将board放在摄像头可见的位置

2. **🖥️ 观察rviz界面**
   - 左侧显示TF树和3D场景
   - 右侧显示检测结果图像
   - 绿色文字表示检测成功，红色表示检测失败

3. **👀 检查检测状态**
   - 在图像窗口中查看"Charuco Corners"数量
   - 至少需要8个角点才能进行可靠的位姿估计
   - 检查"Pose: OK"状态

## 🎯 可视化要素

### TF坐标系层级
```
world (世界坐标系)
└── link_base (机器人基座)
    ├── camera_color_optical_frame (相机光学中心)
    │   └── charuco_board (检测到的ChArUco board)
    └── link6 (机器人末端执行器)
```

### rviz显示内容
- **Grid**: 地面网格参考
- **TF**: 所有坐标系的可视化
- **Image**: 实时检测结果图像
- **PointCloud2**: 可选的深度点云（默认关闭）

## 📊 实时数据监控

### 检查TF变换
```bash
# 查看charuco_board相对于world的位置
ros2 run tf2_ros tf2_echo world charuco_board

# 查看所有可用的TF frames
ros2 run tf2_tools view_frames
```

### 监控检测话题
```bash
# 查看检测结果图像
ros2 topic echo /charuco_pose_publisher_verify/result_image

# 查看相机原始图像
ros2 topic echo /camera/camera/color/image_raw
```

## 🔧 参数调整

### ChArUco Board参数
如果你使用不同尺寸的ChArUco board，需要修改以下参数：

在`charuco_handeye_publish.launch.py`中：
```python
'board_size_x': 7,        # X方向方格数
'board_size_y': 9,        # Y方向方格数  
'square_length': 0.025,   # 方格边长（米）
'marker_length': 0.018,   # ArUco标记边长（米）
```

### 相机参数
如果使用不同的相机，需要：
1. 更新相机标定文件`realsense_calibration_opencv.yaml`
2. 修改图像话题名称
3. 更新相机frame名称

## 🚨 故障排除

### 1. 检测不到ChArUco board
- ✅ 确保board完全在相机视野内
- ✅ 检查lighting条件，避免强光反射
- ✅ 确保board平整，没有折痕或变形
- ✅ 调整相机到board的距离（建议0.3-1.5米）

### 2. TF变换不稳定
- ✅ 确保相机标定准确
- ✅ 检查ChArUco board的print质量
- ✅ 避免相机抖动
- ✅ 确保足够的ArUco标记可见

### 3. rviz中看不到charuco_board frame
- ✅ 检查TF选项中是否启用了`charuco_board`
- ✅ 确认检测器正在运行
- ✅ 查看终端输出的错误信息

### 4. 图像话题连接问题
```bash
# 检查可用的图像话题
ros2 topic list | grep image

# 检查相机是否在运行
ros2 topic hz /camera/camera/color/image_raw
```

## 📍 坐标系说明

### charuco_board Frame的含义
- **原点**: ChArUco board的左上角
- **X轴**: 指向board的右侧
- **Y轴**: 指向board的下方  
- **Z轴**: 垂直于board平面向外

### 在场景中的应用
检测到的`charuco_board` frame提供了：
- ChArUco board在世界坐标系中的准确位置
- board的精确方向（旋转）
- 可以作为机器人抓取、定位等任务的参考点

## 🎮 交互式控制

### 在rviz中的操作
- **旋转视角**: 鼠标左键拖拽
- **缩放**: 鼠标滚轮
- **平移**: 鼠标中键拖拽
- **聚焦**: 点击TF frame名称然后按'F'

### 保存和加载配置
```bash
# 保存当前rviz配置
File -> Save Config As...

# 加载配置
rviz2 -d your_config.rviz
```

## 🔬 高级应用

### 1. 将ChArUco board作为目标对象
可以使用检测到的`charuco_board` frame作为机器人任务的目标：

```python
# 在你的ROS节点中获取ChArUco board位置
import tf2_ros

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer, self)

# 获取ChArUco board相对于机器人基座的位置
transform = tf_buffer.lookup_transform(
    'link_base', 'charuco_board', 
    rclpy.time.Time()
)
```

### 2. 精度验证
使用提供的验证器检查检测精度：

```bash
cd /home/wenhao/uf_custom_ws
python3 handeye_validator_fixed.py
```

### 3. 标定质量评估
监控检测稳定性：
- 位置标准差应 < 1mm（优秀）
- 角度变化应最小
- 检测帧率应稳定

---

## 📝 总结

这个系统为你提供了：
- ✅ 实时ChArUco board检测
- ✅ 准确的6DOF位姿估计
- ✅ 与机器人坐标系的完整集成
- ✅ 直观的rviz可视化
- ✅ 丰富的调试和监控工具

现在你可以将ChArUco board作为场景中的参考对象，用于机器人导航、抓取、定位等各种应用！ 