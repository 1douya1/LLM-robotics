# 笛卡尔倾倒工具 (Cartesian Pour Tool)

## 概述

笛卡尔倾倒工具是对原有倾倒工具的增强版本，提供了精确的位置控制和平滑的笛卡尔运动。这个工具解决了原有工具只能沿Y方向简单移动的限制。

## 主要改进

### 🎯 精确位置控制
- **之前**: 只能沿 -Y 方向相对移动固定距离
- **现在**: 支持指定精确的目标位置 (x, y, z)

### 🛣️ 笛卡尔路径规划
- **之前**: 使用 `MoveRelative` 阶段进行简单的相对移动
- **现在**: 使用 `MoveTo` 阶段配合笛卡尔规划器，提供平滑的直线运动

### 🔄 姿势控制选项
- **保持姿势**: 移动到目标位置时保持当前的末端执行器姿势
- **自定义姿势**: 可以指定目标位置的Roll-Pitch-Yaw角度

### ⚡ 增强的规划器配置
- 更小的步长 (0.005) 确保平滑运动
- 优化的速度和加速度缩放因子
- 更稳定的倾倒序列

## 技术架构

### C++ 后端组件

#### 1. 参数结构 (`CartesianPourTaskParams`)
```cpp
struct CartesianPourTaskParams {
  // 倾倒动作参数
  double tilt_start_deg = 45.0;
  double tilt_end_deg = 120.0;
  double tilt_speed_deg_s = 25.0;
  double pour_hold_sec = 2.0;
  bool plan_only = false;
  
  // 目标位置参数 (base_link坐标系)
  double target_x = 0.1;
  double target_y = -0.5;
  double target_z = 0.13;
  
  // 姿势控制参数
  bool maintain_orientation = true;
  double target_roll = 0.0;   // 仅在maintain_orientation=false时使用
  double target_pitch = 0.0;
  double target_yaw = 0.0;
};
```

#### 2. 任务构建器
- `build_cartesian_pour_task()` - 完整的倾倒任务
- `build_cartesian_pour_only_task()` - 仅倾倒阶段（适用于模块化使用）

#### 3. 任务序列
1. **当前状态** - 获取机械臂当前状态
2. **笛卡尔移动** - 使用CartesianPath规划器移动到目标位置
3. **倾倒序列**:
   - 倾斜到开始角度
   - 倾斜到结束角度
   - 保持倾倒位置
   - 返回到开始角度

### Python 接口

#### `cartesian_pour_to_target()` 函数
```python
def cartesian_pour_to_target(
    target_position: Dict[str, float],           # 必需: {"x": float, "y": float, "z": float}
    tilt_deg: Dict[str, float] = {"start": 45.0, "end": 120.0},
    speed: float = 25.0,
    stop_condition: Dict[str, float] = {"hold_time": 2.0},
    maintain_orientation: bool = True,
    target_orientation: Optional[Dict[str, float]] = None,  # {"roll": float, "pitch": float, "yaw": float}
    plan_only: bool = False,
    timeout_sec: float = 180.0
) -> Dict[str, Any]:
```

## 使用示例

### 基本使用
```python
from ros_client_tools import cartesian_pour_to_target

# 基本倾倒 - 移动到指定位置并保持当前姿势
result = cartesian_pour_to_target(
    target_position={"x": 0.3, "y": -0.4, "z": 0.2},
    tilt_deg={"start": 30.0, "end": 90.0},
    speed=20.0,
    stop_condition={"hold_time": 3.0},
    maintain_orientation=True
)
```

### 高级使用 - 自定义姿势
```python
# 使用自定义姿势倾倒
result = cartesian_pour_to_target(
    target_position={"x": 0.25, "y": -0.35, "z": 0.18},
    tilt_deg={"start": 45.0, "end": 110.0},
    speed=25.0,
    maintain_orientation=False,
    target_orientation={"roll": 0.1, "pitch": 0.0, "yaw": -0.2}
)
```

### MCP 服务器工具
```json
{
  "tool": "cartesian_pour_to_target",
  "arguments": {
    "target_position": {"x": 0.3, "y": -0.4, "z": 0.2},
    "tilt_deg": {"start": 45, "end": 120},
    "speed": 25.0,
    "stop_condition": {"hold_time": 3.0},
    "maintain_orientation": true
  }
}
```

## 坐标系说明

所有位置都是基于 `link_base` (机械臂基座) 坐标系：
- **X轴**: 前进方向 (正值向前)
- **Y轴**: 左右方向 (负值向左，正值向右)
- **Z轴**: 上下方向 (正值向上)

典型的工作空间范围：
- X: 0.1 ~ 0.4m
- Y: -0.6 ~ -0.2m  
- Z: 0.1 ~ 0.3m

## 安装和编译

### 前提条件
- ROS 2 Humble
- MoveIt Task Constructor
- tf2_geometry_msgs

### 编译步骤
```bash
# 在工作空间根目录
cd ~/uf_custom_ws

# 编译包含新功能的包
colcon build --packages-select mtc_tutorial --cmake-args -DCMAKE_BUILD_TYPE=Release

# 设置环境变量
source install/setup.bash
```

## 测试

运行测试脚本来验证功能：

```bash
# 运行基本测试（仅规划模式）
python3 src/mtc_tutorial/scripts/test_cartesian_pour.py
```

测试包括：
1. **基本功能测试** - 验证基础的笛卡尔倾倒功能
2. **自定义姿势测试** - 验证姿势控制功能
3. **完整序列测试** - 验证抓取-倾倒-放置的完整流程
4. **精度控制测试** - 验证多个位置的精确控制

## 与原工具对比

| 特性 | 原倾倒工具 | 笛卡尔倾倒工具 |
|------|-----------|--------------|
| 位置控制 | 仅Y方向相对移动 | 精确的3D位置控制 |
| 路径规划 | MoveRelative | 笛卡尔路径规划 |
| 姿势控制 | 无 | 保持或自定义姿势 |
| 运动平滑度 | 一般 | 高精度平滑运动 |
| 灵活性 | 限制较多 | 高度灵活 |
| 适用场景 | 简单固定倾倒 | 复杂精确倾倒 |

## 故障排除

### 常见问题

1. **规划失败**
   - 检查目标位置是否在工作空间范围内
   - 验证目标姿势是否可达
   - 确认没有碰撞

2. **移动不平滑**
   - 调整笛卡尔规划器步长
   - 降低速度和加速度缩放因子

3. **姿势控制问题**
   - 确认 `maintain_orientation` 设置正确
   - 检查目标姿势的Roll-Pitch-Yaw值是否合理

### 调试模式
使用 `plan_only=True` 进行安全的规划测试，避免实际执行。

## 贡献指南

如需改进或扩展功能：

1. **C++后端修改**: 编辑 `src/mtc_tutorial/src/modular_task_builders.cpp`
2. **Python接口修改**: 编辑 `src/mtc_tutorial/scripts/ros_client_tools.py`
3. **参数调整**: 修改相应的参数结构
4. **添加测试**: 在 `test_cartesian_pour.py` 中增加测试用例

## 许可证

该工具遵循与MTC Tutorial项目相同的许可证。 