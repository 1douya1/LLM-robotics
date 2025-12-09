# MTC Tutorial 使用指南

## 🚀 快速启动

### 1. 启动机器人仿真环境（MTC专用版本）
```bash
# 终端1: 启动UF850机器人仿真（自动加载Gazebo_MTC.rviz配置）
ros2 launch xarm_moveit_config uf850_moveit_gazebo_mtc.launch.py

# 或者使用原版启动文件（加载默认moveit.rviz配置）
# ros2 launch xarm_moveit_config uf850_moveit_gazebo.launch.py add_gripper:=true
```

### 2. 启动MTC演示（使用安全脚本）
```bash
# 终端2: 启动完整的Pick and Place演示
./start_mtc_demo.sh
```

## 🎯 **完整Pick and Place任务流程**

现在MTC Tutorial实现了与官方教程相同的完整任务：

### 📋 **14个任务阶段**
1. **Current State** - 获取机器人当前状态
2. **Move to Ready** - 移动到准备位置（home）
3. **Open Hand** - 打开夹爪

**🔄 PICK 序列：**
4. **Approach Object** - 智能接近物体
5. **Generate Grasp Pose** - 生成多个抓取姿态候选
6. **Allow Collision** - 允许手与物体碰撞
7. **Close Hand** - 关闭夹爪抓取
8. **Attach Object** - 将物体附着到手上
9. **Lift Object** - 提起物体

**📍 PLACE 序列：**
10. **Generate Place Pose** - 生成放置位置
11. **Open Hand** - 打开夹爪
12. **Forbid Collision** - 恢复碰撞检测
13. **Detach Object** - 分离物体
14. **Retreat** - 后退离开
15. **Return Home** - 返回初始位置

### 🔧 **技术特性**
- ✅ **OMPL规划器**：高质量路径规划
- ✅ **智能抓取**：自动生成多个抓取角度
- ✅ **碰撞管理**：动态启用/禁用碰撞检测
- ✅ **物体跟踪**：正确处理物体附着关系
- ✅ **笛卡尔规划**：精确的直线运动
- ✅ **UF850适配**：完全适配UF850机器人配置

## 🔧 多节点冲突问题解决方案

### 问题原因
- **多次启动**：重复运行launch文件而没有完全清理之前的进程
- **节点名称冲突**：多个同名的`mtc_tutorial_node`节点同时运行
- **资源竞争**：多个节点尝试连接同一个action服务器

### 解决方案
1. **自动清理脚本**：`start_mtc_demo.sh`会自动清理旧节点
2. **节点名称统一**：确保launch文件和C++代码中的节点名称一致
3. **命名空间管理**：保持在根命名空间，确保与move_group正常通信

### 手动清理方法
如果遇到节点冲突，手动清理：
```bash
# 清理MTC相关节点
pkill -f mtc_tutorial

# 检查节点是否清理干净
ros2 node list | grep mtc

# 等待2秒后重新启动
sleep 2
```

## 📋 故障排除

### 执行错误代码99999
**症状**：`Task execution failed with error code: 99999`
**原因**：轨迹质量问题或控制器连接问题
**解决**：
1. 检查控制器状态：`ros2 control list_controllers`
2. 现在使用OMPL规划器，执行成功率大幅提升
3. 确保Gazebo_MTC.rviz正确加载

### 多节点冲突
**症状**：看到多个同名节点
**解决**：使用提供的`start_mtc_demo.sh`脚本

### 规划失败
**症状**：`Task planning failed`
**解决**：
1. 检查物体位置是否在机器人工作空间内
2. 验证SRDF预设状态（home, open, close）
3. 确认OMPL配置正确加载

## 🎯 最佳实践

1. **使用提供的启动脚本**：避免手动启动可能导致的冲突
2. **按顺序启动**：先启动机器人仿真，再启动MTC
3. **完全停止后重启**：避免在旧进程还在运行时启动新的
4. **检查系统状态**：启动前确认必要的服务都在运行
5. **使用MTC专用RViz配置**：获得最佳的可视化体验

## 📁 文件说明

- `start_mtc_demo.sh`：安全启动脚本，包含自动清理
- `pick_place_demo.launch.py`：简化的launch文件，避免节点冲突
- `mtc_tutorial.cpp`：**完整的pick and place实现**，与官方教程功能对等
- `uf850_moveit_gazebo_mtc.launch.py`：**新增**专门的MTC启动文件，自动加载Gazebo_MTC.rviz配置
- `uf850_moveit_gazebo.launch.py`：原版启动文件，保持默认moveit.rviz配置

## 🎨 RViz配置说明

### MTC专用配置（推荐）
- 文件：`Gazebo_MTC.rviz`
- 启动：`ros2 launch xarm_moveit_config uf850_moveit_gazebo_mtc.launch.py`
- 特点：包含MTC任务构造器的可视化面板和配置

### 默认MoveIt配置
- 文件：`moveit.rviz`
- 启动：`ros2 launch xarm_moveit_config uf850_moveit_gazebo.launch.py add_gripper:=true`
- 特点：标准的MoveIt Motion Planning界面

## ✅ 验证正常运行

### 简单测试成功标志：
```
[INFO] Task execution completed successfully!
```

### 完整pick and place成功标志：
你将看到机器人依次执行：
1. 移动到准备位置
2. 打开夹爪
3. 接近并抓取圆柱形物体
4. 提起物体
5. 移动到放置位置
6. 放下物体
7. 返回初始位置

## 🚀 与官方教程的对比

| 特性 | 官方教程（Franka） | 我们的实现（UF850） |
|------|-------------------|-------------------|
| 规划器 | ✅ OMPL | ✅ OMPL |
| 任务阶段 | ✅ 完整14阶段 | ✅ 完整15阶段 |
| 抓取姿态生成 | ✅ 支持 | ✅ 支持 |
| 碰撞管理 | ✅ 动态管理 | ✅ 动态管理 |
| 物体附着 | ✅ 支持 | ✅ 支持 |
| 笛卡尔规划 | ✅ 支持 | ✅ 支持 |

这表明MTC任务执行成功，没有节点冲突问题，并且实现了与官方教程相同水平的功能！🎉 