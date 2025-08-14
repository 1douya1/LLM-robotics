# MTC轨迹执行问题解决方案

## 问题描述

当前遇到的错误：
```
[ERROR] Time between points 0 and 1 is not strictly increasing, it is 0.000000 and 0.000000 respectively
[ERROR] Goal was rejected by server
```

这个错误表明MTC生成的轨迹中有两个或多个点具有相同的时间戳，这违反了轨迹执行器的要求。

## 问题原因

1. **Connect阶段的轨迹生成**：Connect阶段可能生成了没有正确时间参数化的轨迹
2. **规划器配置**：某些规划器可能没有自动添加时间参数化
3. **阶段转换**：不同阶段之间的轨迹连接可能导致时间戳问题

## 临时解决方案

### 方案1：分步执行（推荐）

不使用MTC的自动执行，而是手动提取每个阶段的轨迹并分别执行：

```cpp
// 在mtc_tutorial.cpp中修改执行部分
bool execute_enabled = false;  // 禁用自动执行

// 规划完成后，手动执行每个阶段
if (task_.solutions().size() > 0) {
    // 使用MoveGroupInterface手动执行
    moveit::planning_interface::MoveGroupInterface move_group(node_, "uf850");
    moveit::planning_interface::MoveGroupInterface gripper(node_, "uf850_gripper");
    
    // 1. 先打开夹爪
    gripper.setNamedTarget("open");
    gripper.move();
    
    // 2. 移动到抓取位置
    // ... 从解决方案中提取目标位置
    
    // 3. 抓取
    gripper.setNamedTarget("close");
    gripper.move();
    
    // 等等...
}
```

### 方案2：修改规划器配置

在`ompl_planning.yaml`中为每个规划器添加时间参数化：

```yaml
uf850:
  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0
    longest_valid_segment_fraction: 0.01
    # 添加时间参数化
    simplify_solution: true
    interpolate: true
```

### 方案3：使用不同的执行策略

修改Connect阶段使用不同的合并模式：

```cpp
stage->setMergeMode(mtc::stages::Connect::MergeMode::SEQUENTIAL);
```

## 长期解决方案

1. **升级MTC版本**：检查是否有新版本修复了这个问题
2. **自定义时间参数化**：为MTC添加自定义的轨迹后处理步骤
3. **使用Pilz规划器**：Pilz工业运动规划器通常生成时间参数化更好的轨迹

## 调试步骤

1. 运行规划但不执行：
   ```bash
   # 修改代码中的 execute_enabled = false
   ros2 launch mtc_tutorial pick_place_demo.launch.py
   ```

2. 在RViz中查看规划结果

3. 检查每个阶段的轨迹：
   - 查看Motion Planning Tasks面板
   - 检查每个阶段的成本和状态

4. 如果规划看起来正确，可以尝试手动执行

## 已知的工作配置

- 规划：✓ 正常工作
- Gripper执行：✓ 正常工作
- 主臂执行：✗ 时间戳问题

## 后续行动

1. 检查是否有MTC的更新或补丁
2. 考虑向MTC开发者报告这个问题
3. 实现自定义的轨迹执行器 