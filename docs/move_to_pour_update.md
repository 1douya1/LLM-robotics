# 移动到倾倒位置任务构建器 (Move-to-Pour Task Builder)

## 概述

`build_move_to_pour_task` 是一个专门设计用于将机械臂从当前位置移动到倾倒位置的任务构建器。该构建器基于现有的移动功能进行了优化，支持3D位置移动，同时保持当前姿势不变。

## 主要特性

- **姿势保持**: 保持机械臂当前的抓取姿势，只改变位置
- **双模式支持**: 支持绝对位置移动和相对位置移动两种模式
- **平滑运动**: 使用慢速笛卡尔规划器确保平滑、安全的运动
- **灵活参数**: 可调节的速度、加速度和运动参数

## 使用场景

此构建器特别适用于以下场景：
- 机械臂已经抓取了容器，需要移动到倾倒位置
- 需要精确控制移动路径，避免碰撞
- 从抓取位置到倾倒位置的过渡运动

## 参数配置

### MoveToPourTaskParams 结构体

```cpp
struct MoveToPourTaskParams {
  // 绝对位置参数（base_link坐标系）
  double target_x = 0.1;
  double target_y = -0.5;
  double target_z = 0.2;
  
  // 模式选择
  bool use_absolute_position = true;   // true: 绝对位置, false: 相对移动
  
  // 相对移动参数 (当use_absolute_position=false时使用)
  double relative_x = 0.0;
  double relative_y = -0.1;  // 默认向-Y方向移动10cm
  double relative_z = 0.0;
  double relative_min_distance = 0.05;
  double relative_max_distance = 0.15;
  
  // 运动参数
  double velocity_scaling = 0.15;      // 速度比例因子 (0.05-1.0)
  double acceleration_scaling = 0.3;   // 加速度比例因子 (0.1-1.0)
  double step_size = 0.005;           // 笛卡尔路径步长
  double timeout_sec = 10.0;          // 超时时间
  
  bool plan_only = false;             // 仅规划不执行
};
```

## 工作模式

### 模式1: 绝对位置移动 (use_absolute_position = true)

机械臂移动到指定的世界坐标位置：

```cpp
MoveToPourTaskParams params;
params.target_x = 0.2;           // X坐标
params.target_y = -0.6;          // Y坐标  
params.target_z = 0.25;          // Z坐标
params.use_absolute_position = true;
```

**优点**: 
- 精确的位置控制
- 适合已知目标位置的场景

**适用场景**: 
- 倾倒位置是固定的
- 需要移动到特定的世界坐标

### 模式2: 相对移动 (use_absolute_position = false)

基于当前位置进行相对移动：

```cpp
MoveToPourTaskParams params;
params.use_absolute_position = false;
params.relative_x = 0.0;         // X轴相对移动量
params.relative_y = -0.15;       // Y轴相对移动量（向-Y方向15cm）
params.relative_z = 0.05;        // Z轴相对移动量（向上5cm）
```

**优点**: 
- 不依赖绝对坐标
- 适合基于当前位置的增量移动
- 更好地适应不同的起始位置

**适用场景**: 
- 倾倒位置相对于抓取位置有固定偏移
- 需要基于当前位置进行调整

## 使用示例

### C++ 使用示例

```cpp
#include <mtc_tutorial/modular_task_builders.hpp>

// 示例1: 绝对位置移动
MoveToPourTaskParams absolute_params;
absolute_params.target_x = 0.15;
absolute_params.target_y = -0.55; 
absolute_params.target_z = 0.22;
absolute_params.use_absolute_position = true;
absolute_params.velocity_scaling = 0.12;

auto task = build_move_to_pour_task(node, absolute_params);

// 示例2: 相对移动
MoveToPourTaskParams relative_params;
relative_params.use_absolute_position = false;
relative_params.relative_y = -0.12;  // 向-Y方向移动12cm
relative_params.relative_z = 0.03;   // 向上移动3cm
relative_params.velocity_scaling = 0.1;

auto relative_task = build_move_to_pour_task(node, relative_params);
```

### Python 使用示例（通过MTC服务器）

```python
# 绝对位置模式
absolute_request = {
    "action": "move_to_pour",
    "parameters": {
        "target_x": 0.2,
        "target_y": -0.6,
        "target_z": 0.25,
        "use_absolute_position": True,
        "velocity_scaling": 0.15,
        "timeout_sec": 15.0
    }
}

# 相对移动模式
relative_request = {
    "action": "move_to_pour", 
    "parameters": {
        "use_absolute_position": False,
        "relative_x": 0.0,
        "relative_y": -0.15,
        "relative_z": 0.05,
        "velocity_scaling": 0.12
    }
}
```

## 运动参数调节指南

### 速度参数 (velocity_scaling)
- **保守值**: 0.05-0.1 (非常慢，适合精密操作)
- **标准值**: 0.1-0.2 (平衡速度和安全性)
- **快速值**: 0.2-0.5 (较快，需要确保安全)

### 加速度参数 (acceleration_scaling) 
- **平滑值**: 0.1-0.3 (平滑启动和停止)
- **标准值**: 0.3-0.5 (正常加速度)
- **快速值**: 0.5-1.0 (快速响应，可能有冲击)

### 步长参数 (step_size)
- **精细值**: 0.002-0.005 (高精度，计算量大)
- **标准值**: 0.005-0.01 (平衡精度和性能)
- **粗糙值**: 0.01-0.02 (快速计算，精度较低)

## 测试

使用提供的测试脚本验证功能：

```bash
python3 src/mtc_tutorial/scripts/test_move_to_pour.py
```

该脚本提供以下测试选项：
1. 绝对位置模式测试
2. 相对移动模式测试  
3. 仅规划模式测试
4. 全部测试

## 注意事项

1. **坐标系**: 所有位置都相对于`link_base`坐标系
2. **姿势保持**: 笛卡尔规划器会尽量保持当前姿势，但可能有微小调整
3. **碰撞检查**: 确保目标位置和路径无碰撞
4. **速度安全**: 建议从较低的速度参数开始测试
5. **相对移动精度**: 相对移动的精度取决于当前位置的准确性

## 与其他任务的集成

此构建器可以与其他模块化任务构建器组合使用：

```cpp
// 典型的倾倒工作流程
auto pick_task = build_pick_task(node, pick_params);           // 1. 抓取
auto move_task = build_move_to_pour_task(node, move_params);   // 2. 移动到倾倒位置
auto pour_task = build_pour_only_task(node, pour_params);      // 3. 执行倾倒
```

## 故障排除

### 常见问题
1. **任务规划失败**: 检查目标位置是否可达
2. **运动不平滑**: 调低速度和加速度参数
3. **姿势变化过大**: 使用相对移动模式或调整目标位置
4. **超时错误**: 增加timeout_sec参数

### 调试技巧
- 使用`plan_only = true`先测试规划
- 逐步增加移动距离测试
- 检查RViz中的可视化标记
- 查看MoveIt日志获取详细错误信息 