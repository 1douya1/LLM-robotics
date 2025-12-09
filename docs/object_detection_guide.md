# find_object_2d 物体检测使用指南

## 概述
本指南将帮助您使用 find_object_2d 检测物体并获取精准的相机坐标，为后续转换到世界坐标做准备。

## 前提条件
- ✅ 手眼标定已完成并正在发布 TF 变换
- ✅ 深度相机正在发布图像和深度信息
- ✅ 相机话题：
  - RGB: `/camera/camera/color/image_raw`
  - 深度: `/camera/camera/depth/image_rect_raw`
  - 相机信息: `/camera/camera/color/camera_info`

## 使用步骤

### 1. 启动物体检测
```bash
# 在 uf_custom_ws 目录下
ros2 launch object_detection.launch.py
```

这将启动 find_object_2d 并打开图形界面。

### 2. 训练物体模型
1. 在 find_object_2d 的图形界面中，将要检测的物体放在相机前
2. 点击 "Capture" 按钮捕获物体图像
3. 框选物体的特征区域
4. 保存物体模型（会自动保存到 `~/objects` 目录）
5. 重复此过程可以添加更多物体

### 3. 检测物体
物体模型训练完成后，find_object_2d 会自动检测相机视野中的已知物体。

### 4. 获取检测结果
find_object_2d 发布以下话题：

#### 主要话题
- `/objects` (find_object_2d/ObjectsStamped): 2D 检测结果
- `/objectsStamped` (find_object_2d/ObjectsStamped): 带时间戳的检测结果
- `/detection_info` (find_object_2d/DetectionInfo): 详细检测信息

#### TF 框架（3D 位姿）
当启用 PnP 算法时，会发布 TF 变换：
- 框架名称: `object_<物体ID>`
- 父框架: `camera_color_optical_frame`

### 5. 监听检测结果示例
```bash
# 查看 2D 检测结果
ros2 topic echo /objects

# 查看详细检测信息
ros2 topic echo /detection_info

# 查看 TF 变换
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo camera_color_optical_frame object_1
```

## 坐标系说明

### 相机坐标系
- 原点：相机光学中心
- X 轴：向右
- Y 轴：向下
- Z 轴：向前（深度方向）

### 物体坐标
- 通过 TF 变换获取物体在相机坐标系中的位姿
- 位置：(x, y, z) 单位米
- 姿态：四元数 (qx, qy, qz, qw)

## 下一步：坐标转换
有了物体在相机坐标系中的位姿，结合手眼标定结果，可以转换到机器人基坐标系：

```
物体在基坐标系 = 基到相机变换 × 物体在相机坐标系
```

## 常用参数调整

### 启动参数
```bash
# 无图形界面模式
ros2 launch object_detection.launch.py gui:=false

# 自定义物体存储路径
ros2 launch object_detection.launch.py objects_path:="/path/to/your/objects"

# 调整同步模式
ros2 launch object_detection.launch.py approx_sync:=false
```

### 检测参数优化
在图形界面中可以调整：
- 特征检测算法 (SIFT, SURF, ORB 等)
- 匹配阈值
- 最小内点数量
- RANSAC 参数

## 故障排除

### 常见问题
1. **没有检测到物体**
   - 检查物体模型是否正确训练
   - 调整特征检测参数
   - 确保光照条件良好

2. **检测不稳定**
   - 增加训练图像数量
   - 从多个角度训练物体
   - 调整匹配阈值

3. **TF 变换不准确**
   - 检查相机标定质量
   - 确保深度图像质量良好
   - 调整 PnP 算法参数

### 调试命令
```bash
# 检查话题发布
ros2 topic list | grep object

# 查看节点状态
ros2 node info /find_object_2d

# 检查相机话题
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

## 配置文件
系统设置保存在：`~/.ros/find_object_2d.ini`
物体模型保存在：`~/objects/` 