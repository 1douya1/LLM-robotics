# Pick Object 规划优化 - 解决长时间规划问题

## 问题诊断

根据RViz显示，`pick object` 阶段花费了35秒以上仍未找到解决方案，这表明约束条件过于严格。

## 根本原因分析

### 1. **IK求解约束过严**
- 原始设置：`setMaxIKSolutions(3)`, `setMinSolutionDistance(0.5)`
- 复杂的抓取框架变换增加了求解难度

### 2. **距离约束过紧**
- 接近距离：`setMinMaxDistance(0.001, 0.05)` 范围太小
- 抓取框架偏移距离过小：`0.0001m`

### 3. **规划时间不足**
- IK超时：仅5秒
- Connect阶段超时：15秒
- 整体规划器超时：5秒

### 4. **物体位置不理想**
- 位置在机器人工作空间边缘 `(0.0, -0.5, 0.12)`
- 增加了到达难度

## 优化措施

### ✅ 1. **大幅放松IK约束**
```cpp
// 增加IK解数量
wrapper->setMaxIKSolutions(8);  // 从3增加到8

// 大幅减少最小解距离
wrapper->setMinSolutionDistance(0.1);  // 从0.5减少到0.1

// 增加IK超时时间
wrapper->setTimeout(10.0);  // 从5秒增加到10秒
```

### ✅ 2. **简化抓取框架变换**
```cpp
// 使用身份变换，避免复杂旋转
Eigen::Isometry3d grasp_frame_transform;
grasp_frame_transform.setIdentity();

// 仅微调Z偏移
grasp_frame_transform.translation().z() = 0.005;
```

### ✅ 3. **放松距离约束**
```cpp
// 大幅增加接近距离范围
stage->setMinMaxDistance(0.0, 0.15);  // 从0.001-0.05扩大到0.0-0.15

// 减小抓取姿态角度增量
stage->setAngleDelta(M_PI / 4);  // 从60度减少到45度，增加姿态密度
```

### ✅ 4. **增加规划时间**
```cpp
// 大幅增加各阶段超时时间
pipeline_planner->setTimeout(15.0);    // 规划器：5秒→15秒
connect_stage->setTimeout(30.0);       // Connect：15秒→30秒
ik_wrapper->setTimeout(10.0);          // IK：5秒→10秒

// 增加解的数量
task_.plan(8);  // 从4个增加到8个解
```

### ✅ 5. **优化物体位置**
```cpp
// 将物体移动到更容易到达的位置
pose.position.x = 0.1;   // 0.0 → 0.1（向前）
pose.position.y = -0.4;  // -0.5 → -0.4（向右）
pose.position.z = 0.15;  // 0.12 → 0.15（向上）
```

## 性能预期改进

### 规划速度
- **预期**: `pick object` 阶段规划时间从35秒+减少到5-15秒
- **原因**: 放松约束条件，更多可行解

### 成功率
- **预期**: IK求解成功率提高3-5倍
- **原因**: 更多IK解，更小的最小距离要求

### 稳定性
- **预期**: 更稳定地找到可行轨迹
- **原因**: 物体位置优化，距离约束放松

## 调试建议

### 1. **监控规划时间**
```bash
# 运行时观察各阶段时间
ros2 run mtc_tutorial mtc_tutorial
```

### 2. **如果仍然太慢**
进一步放松约束：
```cpp
// 极度放松模式
wrapper->setMaxIKSolutions(16);
wrapper->setMinSolutionDistance(0.05);
stage->setMinMaxDistance(0.0, 0.3);
```

### 3. **物体位置调试**
尝试更容易的位置：
```cpp
// 机器人前方中心位置
pose.position.x = 0.3;
pose.position.y = 0.0;
pose.position.z = 0.2;
```

### 4. **分步测试**
```cpp
// 先测试仅到预抓取位置
bool test_approach_only = true;
if (test_approach_only) {
    // 临时移除复杂阶段，仅测试接近
}
```

## 预期结果

经过这些优化，`pick object` 阶段应该能够：

1. **快速找到解决方案** (5-15秒内)
2. **提供多个可选轨迹** (8个解)
3. **更稳定的IK求解**
4. **更平滑的接近轨迹**

## 进一步优化

如果问题仍然存在，考虑：

### 1. **使用预定义抓取姿态**
```cpp
// 手动定义几个已知可行的抓取姿态
geometry_msgs::msg::PoseStamped predefined_grasp;
// ... 设置已知可行的姿态
```

### 2. **分阶段规划**
```cpp
// 先规划到物体附近，再规划精确抓取
auto approach_stage = /* 粗略接近 */;
auto precise_grasp = /* 精确抓取 */;
```

### 3. **使用不同的规划器**
```cpp
// 尝试Pilz工业运动规划器
auto pilz_planner = std::make_shared<mtc::solvers::PipelinePlanner>();
pilz_planner->setPlannerId("PTP");  // Point-to-Point
```

现在可以测试新的配置，应该显著改善 `pick object` 阶段的规划速度！ 