# OMPL规划器时间戳问题修复指南

## 问题描述

根据[GitHub Issue #578](https://github.com/moveit/moveit_task_constructor/issues/578#issuecomment-2276997444)和[Issue #624](https://github.com/moveit/moveit_task_constructor/issues/624)，MTC在使用OMPL规划器时会出现时间戳错误：

```
[ERROR] Time between points 0 and 1 is not strictly increasing, it is 0.000000 and 0.000000 respectively
```

## 已实施的修复措施

### 1. 节点参数覆盖（基于robotics-qc的建议）

在`main`函数中确保设置：
```cpp
rclcpp::NodeOptions options;
options.automatically_declare_parameters_from_overrides(true);
```

### 2. 请求适配器配置

在构造函数中声明并设置OMPL请求适配器，确保包含`AddTimeOptimalParameterization`：

```cpp
node_->declare_parameter("ompl.request_adapters", 
    "default_planner_request_adapters/AddTimeOptimalParameterization "
    "default_planner_request_adapters/FixWorkspaceBounds "
    "default_planner_request_adapters/FixStartStateBounds "
    "default_planner_request_adapters/FixStartStateCollision "
    "default_planner_request_adapters/FixStartStatePathConstraints");
```

### 3. OMPL规划器配置

在`createTask`中添加了额外的OMPL属性：

```cpp
pipeline_planner->setProperty("goal_joint_tolerance", 1e-4);
pipeline_planner->setProperty("simplify_solutions", true);
pipeline_planner->setProperty("minimum_waypoint_count", 2);
```

### 4. 执行参数调整

使用更宽松的执行参数：
```cpp
trajectory_execution.allowed_execution_duration_scaling: 3.0
trajectory_execution.allowed_goal_duration_margin: 2.0
trajectory_execution.allowed_start_tolerance: 0.05
```

## 使用方法

1. **编译**：
   ```bash
   cd ~/uf_custom_ws
   colcon build --packages-select mtc_tutorial
   source install/setup.bash
   ```

2. **运行测试**：
   ```bash
   # 终端1：启动UF850环境
   ros2 launch xarm_moveit_config uf850_moveit_gazebo.launch.py add_gripper:=true
   
   # 终端2：运行修复后的OMPL版本
   ros2 launch mtc_tutorial pick_place_demo.launch.py
   ```

## 验证步骤

1. **检查适配器加载**：
   在终端输出中查找：
   ```
   Request adapters from move_group: ...AddTimeOptimalParameterization...
   ```

2. **监控执行结果**：
   - 如果看到"Task execution completed successfully!"，说明修复成功
   - 如果仍然有时间戳错误，查看详细的错误信息

## 备选方案

如果OMPL修复不成功，可以使用Pilz规划器版本：
```bash
ros2 launch mtc_tutorial pick_place_demo_pilz.launch.py
```

## 调试建议

1. **检查参数是否正确设置**：
   ```bash
   ros2 run mtc_tutorial check_moveit_config.py
   ```

2. **查看move_group日志**：
   检查是否有关于请求适配器的警告或错误

3. **在RViz中验证**：
   即使执行失败，也可以在RViz中查看规划的轨迹是否合理

## 技术背景

问题的根源在于OMPL规划器生成的路径需要时间参数化。`AddTimeOptimalParameterization`适配器负责为轨迹添加时间信息，但在某些情况下可能：

1. 未被正确加载
2. 参数配置不当
3. 与MTC的集成存在问题

## 社区反馈

根据GitHub讨论，这是一个已知问题，影响多个用户。主要解决方案包括：

1. 确保参数正确覆盖（本修复的核心）
2. 使用Pilz规划器作为替代
3. 手动处理轨迹时间参数化（高级方案）

## 参考资料

- [MTC Issue #578](https://github.com/moveit/moveit_task_constructor/issues/578)
- [MTC Issue #624](https://github.com/moveit/moveit_task_constructor/issues/624)
- [ROS Answers: Trajectory time parameterization](https://answers.ros.org/question/253004/)

## 结论

这个修复综合了社区建议的多种方法，特别是确保：
1. 节点参数能正确覆盖默认值
2. 时间参数化适配器被正确加载
3. OMPL规划器配置合理

如果仍有问题，建议向MTC开发者报告具体的错误信息，以帮助改进框架。 