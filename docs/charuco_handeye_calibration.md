# Charuco手眼标定完整指南

基于您的RealSense相机和UF850机械臂的eye-to-hand（相机固定在基座）手眼标定解决方案。

## 1. 系统概述

### 1.1 配置说明
- **标定类型**: Eye-to-hand (eye_on_base)
- **相机**: RealSense D435i，固定在机器人基座附近
- **标定板**: Charuco板（7x9格，25mm格子，18mm ArUco标记）
- **机械臂**: UF850，Charuco板固定在末端执行器上

### 1.2 坐标系关系
```
base_link (机器人基座)
    ↓ (要标定的变换)
camera_color_optical_frame (相机光学坐标系)
    ↓ (由相机检测)
charuco_board (Charuco板坐标系，固定在末端执行器)
    ↓ (已知的机械臂运动学)
uf850_end_effector (末端执行器坐标系)
```

## 2. 准备工作

### 2.1 硬件准备
1. **Charuco标定板制作**
   - 尺寸：7x9格
   - 格子边长：25mm
   - ArUco标记边长：18mm
   - 使用DICT_4X4_50字典
   - 打印在硬质平板上（建议厚度>5mm）

2. **固定装置**
   - 制作或使用现有夹具将Charuco板牢固固定在UF850末端执行器上
   - 确保板面垂直于末端执行器的主轴
   - 板子应该清晰可见，不被机械臂遮挡

3. **相机安装**
   - RealSense相机固定在机器人工作范围内的稳定位置
   - 确保能够清晰观察到机械臂的工作区域
   - 避免强光直射和反光

### 2.2 软件准备
1. **依赖检查**
```bash
# 检查必要的ROS2包
ros2 pkg list | grep -E "(easy_handeye2|tf2_ros|cv_bridge)"

# 检查Python依赖
python3 -c "import cv2, numpy, yaml, rclpy; print('所有依赖都已安装')"
```

2. **相机标定文件**
   - 确保已运行`charuco_calibration.py`完成相机标定
   - 确认存在`realsense_calibration_opencv.yaml`文件

## 3. 标定流程

### 3.1 启动RealSense相机
```bash
# 启动RealSense相机节点
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    color_width:=640 \
    color_height:=480 \
    color_fps:=30
```

### 3.2 启动机械臂
```bash
# 启动UF850机械臂（根据您的配置调整）
ros2 launch xarm_moveit_config xarm_moveit_demo.launch.py \
    robot_type:=uf850
```

### 3.3 启动手眼标定
```bash
# 启动Charuco手眼标定系统
ros2 launch charuco_handeye_calibration.launch.py \
    calibration_name:=charuco_eye_to_hand \
    image_topic:=/camera/camera/color/image_raw \
    camera_frame:=camera_color_optical_frame \
    robot_base_frame:=base_link \
    robot_effector_frame:=uf850_end_effector \
    calibration_file:=realsense_calibration_opencv.yaml
```

### 3.4 数据采集过程

#### 3.4.1 检查系统状态
1. **检查TF关系**
```bash
# 查看TF树，确认所有坐标系都在发布
ros2 run tf2_tools view_frames.py

# 检查特定变换
ros2 run tf2_ros tf2_echo base_link uf850_end_effector
ros2 run tf2_ros tf2_echo camera_color_optical_frame charuco_board
```

2. **检查Charuco检测**
```bash
# 查看Charuco位姿发布器输出
ros2 topic echo /tf --filter "transforms[0].child_frame_id == 'charuco_board'"
```

#### 3.4.2 标定姿态采集
1. **使用RQT界面**
   - 标定启动后会自动打开RQT标定界面
   - 界面显示当前检测到的Charuco板位姿
   - 显示机械臂当前位姿

2. **移动机械臂采集数据**
   - **重要原则**：最大化旋转变化，最小化平移变化
   - 在同一大致位置，改变末端执行器的朝向
   - 确保每个姿态都能稳定检测到Charuco板

3. **推荐采集序列**（15-20个姿态）：
   ```
   位置1（中心区域）：
   - 垂直向下（0°）
   - 倾斜30°（4个方向）
   - 倾斜60°（4个方向）
   
   位置2（左侧）：
   - 重复上述角度变化
   
   位置3（右侧）：
   - 重复上述角度变化
   ```

4. **数据质量检查**
   - 每个姿态保持2-3秒稳定
   - 确保Charuco板完全在相机视野内
   - 检测到的角点数量 > 8个
   - RQT界面显示"Ready to sample"

5. **采集操作**
   - 在RQT界面点击"Take sample"
   - 等待确认消息
   - 移动到下一个姿态

### 3.5 标定计算
1. **完成采集**
   - 采集15-20个有效样本后，点击"Compute calibration"
   - 系统将计算手眼变换矩阵

2. **结果评估**
   - 查看重投影误差（应 < 2.0像素）
   - 查看标定结果的物理合理性

3. **保存结果**
   - 点击"Save calibration"
   - 标定结果保存在`~/.ros/easy_handeye/charuco_eye_to_hand.calib`

## 4. 标定结果使用

### 4.1 发布标定结果
```bash
# 在生产环境中发布手眼变换
ros2 launch charuco_handeye_publish.launch.py \
    calibration_name:=charuco_eye_to_hand
```

### 4.2 验证标定精度
```bash
# 启用Charuco检测器进行验证
ros2 launch charuco_handeye_publish.launch.py \
    calibration_name:=charuco_eye_to_hand \
    enable_charuco_publisher:=true
```

### 4.3 TF变换检查
```bash
# 查看标定后的变换关系
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# 查看完整的变换链
ros2 run tf2_ros tf2_echo base_link charuco_board
```

## 5. 故障排除

### 5.1 常见问题

#### 5.1.1 Charuco检测失败
**症状**: 控制台显示"未检测到足够的Aruco标记"
**解决方案**:
- 检查光照条件，避免过强或过弱的光照
- 调整相机与标定板的距离（建议0.3-1.0m）
- 确认标定板清晰度和对比度
- 检查标定板是否有损坏或污迹

#### 5.1.2 位姿估计不稳定
**症状**: Charuco板位姿跳动或不稳定
**解决方案**:
- 检查相机标定质量
- 确保标定板完全平整
- 减少相机抖动
- 调整检测参数（在charuco_pose_publisher.py中）

#### 5.1.3 TF变换错误
**症状**: TF树中缺少某些变换
**解决方案**:
```bash
# 检查所有相关节点是否运行
ros2 node list | grep -E "(charuco|handeye|tf)"

# 检查话题是否发布
ros2 topic list | grep -E "(image|tf)"

# 重启相关节点
```

#### 5.1.4 标定精度不够
**症状**: 重投影误差过大（>3.0像素）
**解决方案**:
- 增加采集姿态数量（20-30个）
- 确保姿态变化足够大（旋转>60°）
- 重新进行相机标定
- 检查机械臂运动学标定

### 5.2 调试工具

#### 5.2.1 可视化工具
```bash
# 启动RViz查看坐标系关系
ros2 run rviz2 rviz2

# 在RViz中添加：
# - TF display
# - Image display (/camera/camera/color/image_raw)
# - MarkerArray display (如果有可视化标记)
```

#### 5.2.2 参数调整
如需调整Charuco检测参数，编辑`charuco_pose_publisher.py`：
```python
# 调整检测参数
self.aruco_params.adaptiveThreshWinSizeMin = 3
self.aruco_params.adaptiveThreshWinSizeMax = 23
self.aruco_params.adaptiveThreshWinSizeStep = 10
```

## 6. 高级配置

### 6.1 多相机配置
如果需要使用多台相机，可以为每台相机创建独立的标定：
```bash
ros2 launch charuco_handeye_calibration.launch.py \
    calibration_name:=charuco_eye_to_hand_cam1 \
    image_topic:=/camera1/color/image_raw \
    camera_frame:=camera1_color_optical_frame
```

### 6.2 自动化采集
可以集成MoveIt!规划器来自动移动机械臂采集数据，但需要额外的开发工作。

### 6.3 精度优化
- 使用更高分辨率的相机
- 使用更大尺寸的Charuco板
- 增加采集姿态数量
- 使用更精确的机械臂运动学模型

## 7. 文件清单

- `charuco_pose_publisher.py` - Charuco位姿检测和发布节点
- `charuco_handeye_calibration.launch.py` - 手眼标定启动文件
- `charuco_handeye_publish.launch.py` - 标定结果发布文件
- `realsense_calibration_opencv.yaml` - 相机标定参数文件

记住在使用前确保所有文件都有执行权限：
```bash
chmod +x charuco_pose_publisher.py
chmod +x charuco_handeye_calibration.launch.py
chmod +x charuco_handeye_publish.launch.py
``` 