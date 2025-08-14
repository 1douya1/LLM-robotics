# Move to Pick Connect阶段优化 - 解决长时间等待问题

## 问题分析

### 根本原因
`move to pick` Connect阶段缓慢的核心原因：

1. **状态空间跳跃大**
   - 从 `open hand` 的结束状态到 `generate grasp pose` 生成的预抓取姿态
   - 可能需要大幅度的关节运动

2. **IK解质量问题**
   - GenerateGraspPose生成的某些IK解可能处于关节极限附近
   - 导致Connect阶段难以找到可行路径

3. **单一规划器瓶颈**
   - 原始代码只使用一个规划器
   - 如果该规划器不适合当前问题，会一直等待超时

4. **缺少中间状态**
   - 直接从当前状态跳到抓取姿态
   - 没有中间过渡点帮助规划

## 优化策略

### ✅ 1. **多规划器策略**
```cpp
// 使用3个不同特性的规划器
auto fast_planner = /* RRTConnect - 快速但可能不优 */
auto robust_planner = /* RRTstar - 稳健但较慢 */  
auto pipeline_planner = /* 原始规划器 - 备用 */

// Connect阶段会依次尝试
stage = Connect("move to pick", {
  {arm_group, fast_planner},    // 5秒超时
  {arm_group, robust_planner},  // 10秒超时
  {arm_group, pipeline_planner} // 15秒超时
});
```

**优势**：
- 快速规划器先尝试，成功率70%时可在5秒内完成
- 失败后用更稳健的规划器
- 大幅减少平均规划时间

### ✅ 2. **添加中间路点**
```cpp
// 在open hand后添加
auto stage = MoveTo("move to ready", pipeline_planner);
stage->setGoal("home");  // 移动到标准位置
```

**优势**：
- 将大跳跃分解为两个小跳跃
- home位置通常容易到达
- 从home到抓取位置的路径更容易规划

### ✅ 3. **优化IK解生成**
```cpp
// 简化抓取变换
grasp_frame_transform.setIdentity();
Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());

// 生成更多候选解
wrapper->setMaxIKSolutions(10);  // 增加到10个
wrapper->setMinSolutionDistance(0.05);  // 允许相似解

// 更密集的角度采样
stage->setAngleDelta(M_PI / 6);  // 30度增量
```

**优势**：
- 更简单的抓取姿态更容易到达
- 更多IK解提供更多选择
- Connect阶段可以选择最容易到达的解

### ✅ 4. **限制路径长度**
```cpp
stage->setMaxDistance(1.5);  // 避免绕远路
```

**优势**：
- 防止规划器生成过长的路径
- 加快规划速度

## 性能对比

### 优化前
```
move to pick: 25-35秒 (经常超时)
成功率: ~60%
整体规划时间: 40-60秒
```

### 优化后（预期）
```
move to pick: 3-10秒
成功率: ~95%
整体规划时间: 15-25秒
```

## 调试技巧

### 1. **监控哪个规划器成功**
```cpp
RCLCPP_INFO(LOGGER, "Connect stage completed with planner: %s", 
            stage->getSolutionPlanner());
```

### 2. **分析失败原因**
如果仍然慢，检查：
- 是否有碰撞约束太严格
- 关节限制是否合理
- 起始和目标状态是否太远

### 3. **进一步优化选项**

#### A. 使用关节空间规划
```cpp
// 如果笛卡尔空间规划困难，尝试关节空间
auto joint_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
```

#### B. 预定义安全路点
```cpp
// 定义几个已知安全的中间配置
std::vector<std::string> waypoints = {"ready", "pre_grasp_safe"};
```

#### C. 调整规划器参数
```cpp
// 对RRTConnect优化
planner->setProperty("range", 0.05);  // 减小步长
planner->setProperty("goal_bias", 0.1);  // 增加目标偏向
```

## 关于倒水阶段

倒水阶段复杂的原因：
1. 多个串联的运动阶段
2. 每个阶段都需要满足约束

建议简化：
```cpp
// 使用单个MoveTo代替多个MoveRelative
auto stage = MoveTo("pour position", interpolation_planner);
stage->setGoal(pour_joint_values);  // 预定义的倒水姿态
```

## 实施检查清单

- [x] 实现多规划器策略
- [x] 添加中间路点（move to ready）
- [x] 优化IK解生成参数
- [x] 简化抓取姿态变换
- [x] 设置最大路径长度限制
- [ ] 测试并验证性能改进
- [ ] 根据实际效果微调参数

## 总结

通过这些优化，`move to pick` 阶段应该能够：
1. **快速找到解决方案**（3-10秒）
2. **提高成功率**（95%+）
3. **减少整体规划时间**（50%以上）

关键是将单一的困难规划问题分解为多个简单的子问题，并提供多种求解策略。 