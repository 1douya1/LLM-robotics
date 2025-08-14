# MTC在UF850机械臂上的使用指南

## 系统架构概述

MoveIt Task Constructor (MTC) 是一个用于复杂运动规划任务的框架。在UF850上使用MTC需要理解以下架构：

```
┌─────────────────────┐
│  Gazebo Simulation  │
│   (UF850 + 环境)    │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│     move_group      │
│  (MoveIt核心节点)   │
│  包含MTC Capability │
└──────────┬──────────┘
           │
┌──────────▼──────────┐
│   MTC Tutorial      │
│  (你的任务节点)     │
└─────────────────────┘
```

## 正确的启动顺序

### 1. 首先启动UF850的MoveIt + Gazebo环境

```bash
# 在第一个终端
cd ~/uf_custom_ws
source install/setup.bash
ros2 launch xarm_moveit_config uf850_moveit_gazebo.launch.py
```

这个命令会：
- 启动Gazebo仿真环境
- 加载UF850机器人模型
- 启动move_group节点（包含所有MoveIt配置）
- 启动RViz进行可视化
- 加载MTC的ExecuteTaskSolutionCapability

### 2. 等待系统完全启动

确保看到以下信息：
- "You can start planning now!" (来自move_group)
- RViz显示机器人模型
- Gazebo中机器人正常显示

### 3. 启动MTC任务节点

```bash
# 在第二个终端
cd ~/uf_custom_ws
source install/setup.bash
ros2 launch mtc_tutorial pick_place_demo.launch.py
```

## 关键配置说明

### 1. MoveIt配置自动加载

UF850的所有MoveIt配置都由`MoveItConfigsBuilder`自动加载，包括：

- **URDF/SRDF**: 机器人描述文件
- **kinematics.yaml**: IK求解器配置
  ```yaml
  uf850:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.005
  ```
- **joint_limits.yaml**: 关节限制
  ```yaml
  joint1:
    max_velocity: 2.14
    max_acceleration: 10.0
  ```
- **ompl_planning.yaml**: 路径规划器配置
- **controllers.yaml**: 轨迹执行控制器

### 2. MTC节点配置

MTC节点（mtc_tutorial）会自动从move_group获取配置，包括：
- robot_description
- robot_description_semantic
- planning pipelines
- kinematics配置

### 3. 重要参数

- **use_sim_time**: 仿真环境必须设为true
- **namespace**: 保持为空，确保在根命名空间
- **MTC capability**: move_group必须加载ExecuteTaskSolutionCapability

## 常见问题解决

### 1. "Failed to initialize MTC task"
原因：move_group未正确启动或MTC capability未加载
解决：确保先启动uf850_moveit_gazebo.launch.py

### 2. "No IK solution found"
原因：目标位置不可达或IK求解器配置错误
解决：
- 调整目标位置
- 增加IK尝试次数
- 使用更好的IK求解器（如TRAC-IK）

### 3. "Execution failed"
原因：轨迹执行参数过于严格
解决：在mtc_tutorial.cpp中已设置更宽松的参数：
```cpp
allowed_execution_duration_scaling: 3.0
allowed_goal_duration_margin: 2.0
```

### 4. 参数获取失败
原因：节点启动时机不对
解决：确保move_group完全启动后再启动MTC节点

## 调试技巧

### 1. 检查move_group状态
```bash
ros2 node info /move_group
ros2 param list /move_group
```

### 2. 查看MTC任务状态
在RViz中：
- 添加"Motion Planning Tasks"面板
- 查看任务的各个阶段状态

### 3. 日志级别
```bash
ros2 run mtc_tutorial mtc_tutorial --ros-args --log-level debug
```

## 高级配置

### 1. 更换IK求解器
编辑 `src/xarm_ros2/xarm_moveit_config/config/uf850/kinematics.yaml`:
```yaml
uf850:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
```

### 2. 调整规划器
在代码中设置：
```cpp
pipeline_planner->setPlannerId("RRTstar");  // 或其他OMPL规划器
```

### 3. 自定义抓取策略
修改`createTask()`函数中的抓取生成器参数

## 执行流程

1. **场景设置**: setupPlanningScene()创建要抓取的物体
2. **任务创建**: createTask()构建MTC任务的各个阶段
3. **任务规划**: task.plan()生成解决方案
4. **可视化**: 在RViz中显示规划结果
5. **执行**: task.execute()执行最佳解决方案

## 注意事项

1. **命名空间一致性**: 确保所有节点在相同命名空间
2. **时间同步**: 仿真环境必须使用use_sim_time=true
3. **资源管理**: MTC会消耗大量内存，确保系统资源充足
4. **执行安全**: 在真实机器人上执行前，先在仿真中充分测试

## 进阶开发

如需开发更复杂的任务：
1. 参考MoveIt Task Constructor文档
2. 研究更多的Stage类型
3. 实现自定义的Stage
4. 优化规划器参数以提高成功率 