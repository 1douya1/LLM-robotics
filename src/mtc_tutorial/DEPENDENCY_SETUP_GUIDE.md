# MTC MCP工具依赖设置指南

本指南解决重构后MTC MCP工具的依赖问题，包括 `moveit_commander` 和 `mtc_interface` 的设置。

## 问题概述

重构后的工具遇到两个主要依赖问题：

1. **`moveit_commander` 不可用**：ROS2 Humble中没有传统的moveit_commander包
2. **`mtc_interface` 导入失败**：需要正确的环境设置才能导入本地构建的包

## 解决方案

### 1. moveit_commander 替代方案 ✅

我们已经重构代码使用**ROS2原生接口**替代 `moveit_commander`：

- **旧方式**：使用 `moveit_commander.PlanningSceneInterface()`
- **新方式**：直接使用 ROS2 发布者和服务客户端

```python
# 不再需要 moveit_commander
# import moveit_commander  # ❌ 在ROS2 Humble中不可用

# 使用原生ROS2接口
from moveit_msgs.msg import CollisionObject, PlanningScene  # ✅
from moveit_msgs.srv import GetPlanningScene  # ✅
```

### 2. mtc_interface 环境设置 ✅

#### 方法1：在脚本中自动设置（推荐）

我们已经修改了 `ros_client_tools.py` 来自动设置环境：

```python
# 自动检测workspace并设置Python路径
workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
install_path = os.path.join(workspace_root, "install")
python_path = os.path.join(install_path, "mtc_interface", "local", "lib", "python3.10", "dist-packages")
sys.path.insert(0, python_path)
```

#### 方法2：手动设置环境变量

如果自动设置不起作用，可以手动设置：

```bash
# 在运行测试前，确保source环境
cd /path/to/uf_custom_ws
source install/setup.bash
python3 src/mtc_tutorial/scripts/test_refactored_tools_fixed.py
```

#### 方法3：验证mtc_interface可用性

```bash
cd /path/to/uf_custom_ws
source install/setup.bash
python3 -c "import mtc_interface.action; print('mtc_interface可用')"
```

## 测试依赖修复

### 使用修复版测试脚本

```bash
cd src/mtc_tutorial/scripts
python3 test_refactored_tools_fixed.py
```

修复版测试脚本的改进：
- ✅ 自动设置环境变量和Python路径
- ✅ 包含mtc_interface导入测试
- ✅ 更好的错误处理和诊断信息
- ✅ 详细的环境信息输出

### 期望的测试结果

理想情况下，你应该看到：

```
Found install directory: /path/to/uf_custom_ws/install
Added to Python path: /path/to/uf_custom_ws/install/mtc_interface/local/lib/python3.10/dist-packages
AMENT_PREFIX_PATH: /path/to/uf_custom_ws/install

==================================================
测试 0: mtc_interface导入测试
==================================================
✓ mtc_interface导入成功
✓ mtc_interface导入: 通过

==================================================
测试 1: 场景构建 (setup_planning_scene)
==================================================
Published collision object: table_surface
Published collision object: table_base
Published collision object: object
✓ 场景构建: 通过
```

## 替代的轻量级测试

如果完整测试仍有问题，可以运行单独的组件测试：

### 测试1：验证基本ROS2连接

```python
#!/usr/bin/env python3
import rclpy
from moveit_msgs.msg import CollisionObject

rclpy.init()
node = rclpy.create_node('test_node')
pub = node.create_publisher(CollisionObject, '/collision_object', 10)
print("✓ ROS2和MoveitMsgs可用")
node.destroy_node()
rclpy.shutdown()
```

### 测试2：验证mtc_interface

```python
#!/usr/bin/env python3
import sys
import os

# 设置环境（如果需要）
workspace_root = "/path/to/uf_custom_ws"  # 替换为实际路径
install_path = os.path.join(workspace_root, "install")
python_path = os.path.join(install_path, "mtc_interface", "local", "lib", "python3.10", "dist-packages")
if os.path.exists(python_path):
    sys.path.insert(0, python_path)

try:
    import mtc_interface.action
    print("✅ mtc_interface导入成功")
except ImportError as e:
    print(f"❌ mtc_interface导入失败: {e}")
```

## 构建确认

确保mtc_interface正确构建：

```bash
cd /path/to/uf_custom_ws
colcon build --packages-select mtc_interface
source install/setup.bash
```

验证安装：

```bash
ls install/mtc_interface/local/lib/python3.10/dist-packages/
# 应该看到：mtc_interface/
```

## 故障排除

### 问题1：mtc_interface仍然无法导入

**解决方案**：
1. 确保已运行 `colcon build`
2. 检查 `install/mtc_interface` 目录是否存在
3. 手动source环境：`source install/setup.bash`

### 问题2：规划场景服务不可用

**解决方案**：
1. 确保 `move_group` 节点正在运行
2. 检查服务是否可用：`ros2 service list | grep planning_scene`
3. 启动move_group：`ros2 launch your_robot_moveit_config demo.launch.py`

### 问题3：碰撞对象未出现在RViz

**解决方案**：
1. 在RViz中启用"PlanningScene"显示
2. 确保frame_id正确（通常是"link_base"或"base_link"）
3. 检查topic `/collision_object` 是否有消息：`ros2 topic echo /collision_object`

## 成功标志

当依赖正确设置后，你应该能够：

✅ 导入 `mtc_interface.action` 无错误  
✅ 发布 `CollisionObject` 消息到 `/collision_object`  
✅ 调用 `/move_group/get_planning_scene` 服务  
✅ 在RViz中看到规划场景对象  
✅ 执行MCP工具的complete_pour action（至少planning模式）

## 下一步

一旦依赖问题解决，你可以：

1. 使用新的模块化MCP工具
2. 在Agent中集成这些工具
3. 扩展添加更多场景对象类型
4. 优化工具性能和错误处理

如果仍有问题，请检查：
- ROS2环境是否正确source
- MoveIt配置是否正确加载
- 网络连接和节点通信是否正常 