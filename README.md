# UF850 机械臂视觉抓取与倒水系统

基于 ROS2 Humble 和 MoveIt Task Constructor 的智能机械臂操作系统，支持 AI Agent 通过 MCP 协议控制机器人执行复杂任务。

## 项目简介

这是一个面向 UFACTORY UF850 机械臂的完整机器人操作系统，集成了：
- **视觉感知**：基于 RealSense D435i 深度相机的物体检测和定位
- **手眼标定**：ChArUco 标定板自动手眼标定系统
- **运动规划**：MoveIt Task Constructor (MTC) 任务级运动规划
- **AI 集成**：通过 MCP 协议实现 AI Agent 控制机器人

## 系统架构

### 五层分层架构
```
AI Agent → MCP Server → ROS2 Client → ROS2 Action Server → MTC Task Builder
```

详细架构说明见 [docs/architecture_analysis.md](docs/architecture_analysis.md)

### 核心功能模块

1. **物体检测系统**
   - find_object_2d：基于特征匹配的 2D 物体检测
   - YOLO One-shot：基于深度学习的单次物体检测
   - 自动坐标转换：相机坐标系 → 机器人基座坐标系

2. **手眼标定系统**
   - ChArUco 标定板自动化标定流程
   - TF 变换发布和验证
   - 实时标定精度监控

3. **任务执行系统**
   - Pick：物体抓取任务
   - Place：物体放置任务
   - Pour：倒水动作任务
   - Move：移动到指定位置

4. **AI Agent 集成**
   - LangGraph Agent 框架
   - MCP 协议标准化接口
   - 多模态任务规划

## 目录结构

```
uf_custom_ws/
├── src/                          # ROS2 源代码
│   ├── mtc_tutorial/            # MTC 教程和任务构建器
│   │   ├── src/                 # C++ 源码（任务构建器）
│   │   ├── scripts/             # Python 脚本（MCP服务器等）
│   │   └── launch/              # 启动文件
│   ├── mtc_interface/           # MTC 接口定义
│   ├── xarm_ros2/               # XArm/UF850 机器人驱动
│   ├── find_object_2d/          # 物体检测包
│   ├── easy_handeye2/           # 手眼标定包
│   └── moveit_task_constructor/ # MTC 核心框架
│
├── docs/                         # 文档
│   ├── quick_start_guide.md     # 快速开始指南
│   ├── architecture_analysis.md  # 架构分析
│   ├── oneshot_object_detection.md  # One-shot 检测指南
│   ├── charuco_handeye_calibration.md  # 手眼标定指南
│   └── ...                      # 其他专题文档
│
├── scripts/                      # 工具脚本
│   ├── object_detection.launch.py  # 物体检测启动文件
│   ├── charuco_calibration.py   # ChArUco 标定脚本
│   ├── handeye_validator.py     # 手眼标定验证
│   ├── start_and_test_detection.sh  # 检测系统启动脚本
│   └── start_calibrated_system.sh   # 标定系统启动脚本
│
├── config/                       # 配置文件
│   ├── realsense_calibration.yaml  # RealSense 相机标定
│   └── realsense_calibration_opencv.yaml
│
├── Langraph_Agent/              # AI Agent 模块
│   ├── agent_app.py            # Agent 主程序
│   ├── task_graph.py           # 任务图定义
│   └── simple_backend.py       # 后端服务
│
├── Yolo_RealSense/             # YOLO 检测模块
│   ├── advanced_3d_detection.py  # 3D 检测实现
│   └── README.md               # YOLO 使用说明
│
└── Yolo_pretrain/              # YOLO 训练数据（不提交到git）
```

## 快速开始

### 1. 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- CUDA 11.8+ (可选，用于 YOLO)

### 2. 依赖安装

```bash
# 安装 ROS2 Humble
sudo apt install ros-humble-desktop-full

# 安装 MoveIt2
sudo apt install ros-humble-moveit

# 安装 RealSense 相机驱动
sudo apt install ros-humble-realsense2-camera

# 安装 Python 依赖
pip install ultralytics opencv-python numpy
```

### 3. 编译工作空间

```bash
cd ~/uf_custom_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. 启动系统

#### 方式 1：完整系统启动（推荐用于首次测试）
```bash
# 启动相机 + 手眼标定 + 物体检测
bash scripts/start_calibrated_system.sh
```

#### 方式 2：仅物体检测
```bash
# 使用 One-shot 检测启动脚本
bash scripts/start_and_test_detection.sh
```

#### 方式 3：启动 MTC 任务服务器
```bash
# 启动模块化任务服务器（用于 AI Agent 控制）
ros2 launch mtc_tutorial modular_task_server.launch.py
```

#### 方式 4：启动 AI Agent
```bash
cd Langraph_Agent
bash start_simple.sh
```

### 5. 测试验证

```bash
# 测试物体检测
ros2 service call /trigger_object_detection std_srvs/srv/Trigger

# 查看检测结果
ros2 topic echo /detected_objects --once

# 测试 MCP 工具（需要先启动 modular_task_server）
cd src/mtc_tutorial/scripts
python3 mtc_mcp_server.py
```

## 主要功能使用

### 物体检测与定位

详见：[docs/oneshot_object_detection.md](docs/oneshot_object_detection.md)

### 手眼标定

详见：[docs/charuco_handeye_calibration.md](docs/charuco_handeye_calibration.md)

### MTC 任务规划

详见：[docs/mtc_usage_guide.md](docs/mtc_usage_guide.md)

### AI Agent 控制

详见：[Langraph_Agent/README.md](Langraph_Agent/README.md)

## 技术栈

- **ROS2 Humble**: 机器人操作系统
- **MoveIt2**: 运动规划框架
- **MoveIt Task Constructor (MTC)**: 任务级运动规划
- **FastMCP**: MCP 协议实现
- **OpenCV**: 图像处理
- **YOLOv8**: 深度学习物体检测
- **RealSense SDK**: RGB-D 相机驱动
- **LangGraph**: AI Agent 框架

## 数据流

```
相机图像 → 物体检测 → 坐标转换 → 规划场景更新 → MTC任务规划 → 机器人执行
         (find_object_2d/YOLO)  (TF变换)    (PlanningScene)   (Task Builder)  (XArm API)
```

## 故障排查

常见问题及解决方案见：[docs/troubleshooting_guide.md](docs/troubleshooting_guide.md)

## 开发指南

### 添加新的任务类型

1. 在 `src/mtc_tutorial/src/modular_task_builders.cpp` 中添加任务构建器
2. 在 `src/mtc_tutorial/src/modular_task_server.cpp` 中注册任务类型
3. 在 `src/mtc_tutorial/scripts/ros_client_tools.py` 中添加 ROS2 客户端函数
4. 在 `src/mtc_tutorial/scripts/mtc_mcp_server.py` 中添加 MCP 工具

### 调试技巧

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看话题列表
ros2 topic list

# 实时监控日志
ros2 run rqt_console rqt_console

# 可视化机器人状态
rviz2
```

## 贡献指南

欢迎贡献代码、报告问题或提出改进建议！

## 许可证

TODO: 添加许可证信息

## 联系方式

- 维护者：wenhao
- 项目地址：TODO

## 致谢

本项目基于以下开源项目：
- [MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor)
- [easy_handeye2](https://github.com/IFL-CAMP/easy_handeye)
- [find_object_2d](https://github.com/introlab/find-object)
- [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2)
