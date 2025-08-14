# 倒水位置配置说明

## 功能概述

倒水功能现在使用**指定的固定位置**而不是相对运动，通过笛卡尔运动规划精确移动到倒水位置。

## 倒水位置配置

### 当前倒水位置
```cpp
// 倒水位置坐标
pour_position.pose.position.x = 0.3;   // X坐标：30cm
pour_position.pose.position.y = 0.2;   // Y坐标：20cm（右侧）
pour_position.pose.position.z = 0.25;  // Z坐标：25cm（高度）
```

### 位置说明
- **X = 0.3m**：比物体位置(0.4m)近10cm
- **Y = 0.2m**：在物体右侧20cm
- **Z = 0.25m**：足够高度避免碰撞

## 完整任务流程

1. **物体位置**：x=0.4, y=0.0, z=0.05
2. **预抓取位置**：x=0.35, y=0.0, z=0.15
3. **抓取物体**：接近、抓取、抬起
4. **倒水位置**：x=0.3, y=0.2, z=0.25 ← **新增固定位置**
5. **倒水动作**：45°→90°→保持→45°→0°
6. **放置位置**：x=0.4, y=0.0, z=0.25
7. **返回初始位置**

## 如何调整倒水位置

### 1. 修改X坐标（前后位置）
```cpp
// 更靠近物体
pour_position.pose.position.x = 0.35;  // 35cm

// 更远离物体
pour_position.pose.position.x = 0.25;  // 25cm
```

### 2. 修改Y坐标（左右位置）
```cpp
// 更右侧
pour_position.pose.position.y = 0.3;   // 30cm

// 更左侧
pour_position.pose.position.y = 0.1;   // 10cm
```

### 3. 修改Z坐标（高度）
```cpp
// 更高
pour_position.pose.position.z = 0.3;   // 30cm

// 更低
pour_position.pose.position.z = 0.2;   // 20cm
```

## 优势

### 1. 精确控制
- ✅ 直接指定倒水位置
- ✅ 不依赖相对运动计算
- ✅ 位置可预测和重复

### 2. 更好的规划
- ✅ 使用笛卡尔运动规划
- ✅ 路径更平滑
- ✅ 避免碰撞

### 3. 易于调试
- ✅ 位置参数清晰
- ✅ 容易调整
- ✅ 便于测试

## 测试验证

### 1. 编译和运行
```bash
cd ~/uf_custom_ws
colcon build --packages-select mtc_tutorial
source install/setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py
```

### 2. 观察要点
- **倒水位置**：机器人是否移动到指定位置
- **运动轨迹**：是否平滑到达倒水位置
- **倒水动作**：joint6旋转是否正常
- **终端输出**：查看位置配置信息

### 3. 调试建议
如果倒水位置不合适：
1. 调整X坐标改变前后位置
2. 调整Y坐标改变左右位置
3. 调整Z坐标改变高度
4. 确保位置在机器人工作空间内

## 配置示例

### 示例1：近距离倒水
```cpp
pour_position.pose.position.x = 0.35;  // 更靠近物体
pour_position.pose.position.y = 0.1;   // 稍微右侧
pour_position.pose.position.z = 0.2;   // 较低高度
```

### 示例2：远距离倒水
```cpp
pour_position.pose.position.x = 0.25;  // 远离物体
pour_position.pose.position.y = 0.3;   // 更右侧
pour_position.pose.position.z = 0.3;   // 更高高度
```

### 示例3：中心倒水
```cpp
pour_position.pose.position.x = 0.3;   // 适中距离
pour_position.pose.position.y = 0.0;   // 正前方
pour_position.pose.position.z = 0.25;  // 适中高度
```

## 注意事项

1. **工作空间限制**：确保倒水位置在机器人可达范围内
2. **碰撞避免**：倒水位置不应与其他物体碰撞
3. **姿态保持**：倒水位置应保持合适的末端执行器姿态
4. **路径规划**：确保从抓取位置到倒水位置的路径可行

## 总结

通过使用指定的固定倒水位置，我们实现了：
- ✅ **更精确的位置控制**
- ✅ **更平滑的运动轨迹**
- ✅ **更易于调试和调整**
- ✅ **更可预测的行为**

这个改进使倒水功能更加实用和可靠。 