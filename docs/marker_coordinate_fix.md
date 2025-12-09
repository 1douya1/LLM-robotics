# 标记坐标系修复说明

## 🐛 问题描述
RViz中的物体标记显示在相机坐标系下，而不是基座坐标系下。

## 🔧 已修复的问题

### 1. depth_scale参数错误
- **之前**: `depth_scale = 0.01` (错误，导致深度计算错误)
- **现在**: `depth_scale = 0.001` (正确，RealSense深度单位转换)

### 2. 标记发布逻辑增强
- 添加了`msg.transform_available`检查
- 增加了详细的调试日志
- 更严格的坐标系选择条件

## 🧪 验证修复

### 1. 重新启动检测系统
```bash
# 停止现有的检测节点
# 然后重新启动
ros2 launch mtc_tutorial detection_only.launch.py
```

### 2. 运行测试脚本
```bash
# 在新终端中运行坐标测试
python3 test_marker_coordinates.py
```

### 3. 触发检测
```bash
ros2 service call /trigger_object_detection std_srvs/srv/Trigger
```

### 4. 查看日志输出
应该看到类似这样的输出：
```
=== 检测结果分析 ===
transform_available: True
物体 0 (cup):
  transform_valid: True
  相机坐标: (0.015, 0.088, 0.532)
  基座坐标: (0.098, -0.461, 0.101)

=== 标记分析 ===
标记坐标系: link_base
标记位置: (0.098, -0.461, 0.101)
```

## 🎯 预期结果

修复后，您应该看到：
1. **标记坐标系**应该是`link_base`而不是`camera_color_optical_frame`
2. **标记位置**应该匹配基座坐标而不是相机坐标
3. **RViz中的标记**应该出现在机器人工作空间内的正确位置

## 🔍 故障排除

### 如果标记仍然在相机坐标系：
1. 检查手眼标定TF是否正确发布：
   ```bash
   ros2 run tf2_ros tf2_echo link_base camera_color_optical_frame
   ```

2. 检查检测日志中的transform_valid状态

3. 确认标记发布器参数：
   ```bash
   ros2 param get /object_marker_publisher use_base_frame
   ```

### 如果深度计算仍然不正确：
1. 验证depth_scale参数：
   ```bash
   ros2 param get /oneshot_object_detection depth_scale
   ```
   应该返回0.001

2. 检查相机内参是否正确获取

## 📝 手动修复（如果需要）

如果自动修复不生效，您可以手动强制使用基座坐标系：

```python
# 在object_marker_publisher.py中，临时强制使用基座坐标系
if True:  # 强制使用基座坐标系
    position = obj.position_base
    frame_id = msg.base_frame
    coord_info = "基座坐标系"
```

---

**注意**: 修复后请重新启动所有相关节点以确保参数生效。 