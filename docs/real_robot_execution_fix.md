# UF850真机执行问题修复指南

## 问题分析

根据您的错误日志，主要问题包括：

### 1. 夹爪控制失败
```
[ros2_control_node-8] [ERROR] goal_handle canceled exception
[move_group-4] [WARN] Controller handle xarm_gripper reports status PREEMPTED
```

### 2. 执行被取消 (错误代码 99999)
```
[mtc_tutorial-1] [ERROR] Goal was aborted or canceled
[mtc_tutorial-1] [ERROR] Task execution failed with error code: 99999
```

## 修复措施

### 1. 夹爪控制优化
- ✅ 增加夹爪命令超时时间到10秒
- ✅ 改进错误处理和重试机制

### 2. 运动参数优化（真机专用）
- ✅ 降低速度缩放因子：`0.3`（原来0.5）
- ✅ 降低加速度缩放因子：`0.5`（原来0.7）
- ✅ 降低笛卡尔速度：`0.2`（原来0.6）
- ✅ 增加规划超时时间：`5秒`（原来2秒）

### 3. 执行前检查
- ✅ 增加系统稳定等待时间：`5秒`（原来2秒）
- ✅ 添加机器人状态检查
- ✅ 改进错误诊断和报告

## 使用步骤

### 1. 检查机器人状态
在运行MTC之前，确保：
```bash
# 检查机器人连接状态
ros2 topic echo /xarm/xarm_states

# 检查关节状态
ros2 topic echo /joint_states

# 检查夹爪状态
ros2 topic echo /xarm/gripper_states
```

### 2. 启动系统
```bash
# 启动机器人控制器
ros2 launch xarm_moveit_config uf850_moveit_realmove.launch.py

# 等待系统完全启动后，运行MTC
ros2 run mtc_tutorial mtc_tutorial
```

### 3. 监控执行
- 观察RViz中的轨迹可视化
- 监控控制台输出的详细错误信息
- 检查机器人实际运动是否平滑

## 常见问题排查

### 问题1：夹爪命令失败
**症状：** `gripper reports status PREEMPTED`
**解决方案：**
1. 检查夹爪连接和电源
2. 手动测试夹爪命令：
   ```bash
   ros2 action send_goal /xarm/gripper_action xarm_msgs/action/GripperMove "{position: 0.0, max_effort: 5.0}"
   ```
3. 重新校准夹爪
4. 检查夹爪配置文件

### 问题2：关节运动超限
**症状：** `CONTROL_FAILED` 或关节位置错误
**解决方案：**
1. 检查关节限制配置：
   ```bash
   cat src/xarm_ros2/xarm_moveit_config/config/uf850/joint_limits.yaml
   ```
2. 验证目标位置是否在工作空间内
3. 减少运动幅度和速度

### 问题3：通信超时
**症状：** `goal_handle canceled` 或 `TIMED_OUT`
**解决方案：**
1. 检查网络连接稳定性
2. 增加超时时间
3. 确保机器人控制器正常运行

### 问题4：急停或安全限制
**症状：** `GOAL_ABORTED_OR_CANCELED` (错误代码 99999)
**解决方案：**
1. 检查急停按钮状态
2. 确认安全区域设置
3. 验证碰撞检测配置
4. 检查关节位置和速度限制

## 调试技巧

### 1. 分步执行
修改代码暂时禁用某些阶段：
```cpp
bool execute_enabled = false;  // 仅规划不执行
```

### 2. 简化任务
先测试简单的关节运动：
```cpp
// 只测试打开夹爪
auto stage = std::make_unique<mtc::stages::MoveTo>("test open", interpolation_planner);
stage->setGroup(hand_group_name);
stage->setGoal("open");
```

### 3. 监控关节状态
```bash
# 实时监控关节状态
ros2 topic echo /joint_states --no-arr

# 监控机器人错误
ros2 topic echo /xarm/xarm_states
```

### 4. 查看详细日志
```bash
# 启动时增加日志级别
ros2 launch xarm_moveit_config uf850_moveit_realmove.launch.py --ros-args --log-level DEBUG
```

## 参数调整建议

### 安全参数（保守设置）
```cpp
// 超低速度用于测试
pipeline_planner->setMaxVelocityScalingFactor(0.1);
pipeline_planner->setMaxAccelerationScalingFactor(0.2);

// 增加更多超时时间
stage->setTimeout(15.0);

// 减少运动距离
stage->setMinMaxDistance(0.01, 0.05);
```

### 生产参数（优化后）
```cpp
// 适中速度用于实际应用
pipeline_planner->setMaxVelocityScalingFactor(0.3);
pipeline_planner->setMaxAccelerationScalingFactor(0.5);
```

## 预防措施

1. **定期校准**：定期校准机器人和夹爪
2. **环境检查**：确保工作空间无障碍物
3. **软件更新**：保持ROS2和驱动程序最新
4. **备份配置**：保存工作的配置文件
5. **渐进测试**：从简单到复杂逐步测试功能

## 应急处理

如果机器人卡死或异常：
1. 按下急停按钮
2. 重启机器人控制器
3. 重新启动ROS2节点
4. 检查机器人状态后重新开始 