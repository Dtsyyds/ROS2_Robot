# AGV ROS2 Control Manager

基于 ROS 2 的 AGV（自动导引车）多机器人控制平台，支持双机械臂、底盘运动、视觉感知、路径规划和 MoveIt 运动规划等功能。通过 WebSocket 协议实现远程控制端（Unity 3D / Web 前端）与本地 ROS2 系统的实时通信。

## 📋 目录

- [系统架构](#系统架构)
- [功能包一览](#功能包一览)
- [各功能包详细说明](#各功能包详细说明)
  - [agv_protocol — 通信协议层](#agv_protocol--通信协议层)
  - [agv_bridge — WebSocket 桥接节点](#agv_bridge--websocket-桥接节点)
  - [agv_hardware — 硬件接口层](#agv_hardware--硬件接口层)
  - [agv_description — 机器人模型描述](#agv_description--机器人模型描述)
  - [agv_bringup — 系统启动配置](#agv_bringup--系统启动配置)
  - [agv_moveit_config — MoveIt 运动规划配置](#agv_moveit_config--moveit-运动规划配置)
  - [camera — 相机驱动与视觉](#camera--相机驱动与视觉)
  - [pathplannernode — 路径规划节点](#pathplannernode--路径规划节点)
  - [wall_robot_pkg — 壁面机器人功能包](#wall_robot_pkg--壁面机器人功能包)
  - [realsense-ros — Intel RealSense 相机驱动](#realsense-ros--intel-realsense-相机驱动)
  - [moveit2_calibration — 手眼标定](#moveit2_calibration--手眼标定)
  - [robot_move_ws — 机器人运动工作空间](#robot_move_ws--机器人运动工作空间)
- [通信协议](#通信协议)
- [环境要求与依赖安装](#环境要求与依赖安装)
- [编译与运行](#编译与运行)
- [如何上传到 GitHub](#如何上传到-github)
- [项目结构](#项目结构)

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────┐
│                        远程控制端 (Unity / Web)                       │
└────────────────────────────┬────────────────────────────────────────┘
                             │ WebSocket (JSON)
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  agv_protocol (协议解析)  ←→  agv_bridge (桥接管理器/消息路由)        │
│                                    │                                 │
│                    ┌───────────────┼───────────────┐                 │
│                    ▼               ▼               ▼                 │
│              ChassisHandler   LiftingHandler   ArmHandler ...        │
│                    │               │               │                 │
└────────────────────┼───────────────┼───────────────┼─────────────────┘
                     ▼               ▼               ▼
              ROS2 Topics      ROS2 Topics     ROS2 Topics
              /cmd_vel         /lifting_cmd    /arm/joint_cmd
              /odom            /lifting_state  /arm/joint_states
              /battery_state                                  

┌─────────────────────────────────────────────────────────────────────┐
│  agv_hardware (ros2_control 硬件接口)                                │
│  ├── RobotController (珞石机械臂 SDK)                                │
│  ├── WebSocketClient (向上位机发送关节数据)                           │
│  └── WebSocketServer (接收上位机控制指令)                             │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│  agv_moveit_config (MoveIt2 运动规划)                                │
│  ├── move_group (运动规划组)                                         │
│  ├── ros2_controllers (关节轨迹控制器)                               │
│  └── RViz2 可视化                                                    │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│  感知层                                                              │
│  ├── camera (奥视达相机 + 视频流服务)                                │
│  ├── realsense-ros (Intel RealSense 深度相机)                        │
│  └── pathplannernode (点云处理 + 路径规划)                           │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 功能包一览

| 功能包 | 语言 | 类型 | 说明 |
|--------|------|------|------|
| `agv_protocol` | C++ | 库 | WebSocket 通信协议与 JSON 命令解析 |
| `agv_bridge` | C++ | 节点 | 中介者模式的 WebSocket 桥接节点，消息路由与分发 |
| `agv_hardware` | C++ | 插件 | ros2_control 硬件接口，对接珞石机械臂与 AGV 底盘 |
| `agv_description` | xacro/URDF | 资源 | 机器人 URDF 模型、STL 网格、RViz 配置 |
| `agv_bringup` | XML/YAML | 启动 | 系统级 launch 文件与控制器配置 |
| `agv_moveit_config` | Python/YAML | 配置 | MoveIt2 运动规划全套配置与 launch 文件 |
| `camera` | C++/Python | 节点 | 奥视达(Astras)相机驱动、TF 发布、视频流服务 |
| `pathplannernode` | Python | 节点 | 点云处理、路径规划、注意力路径生成 |
| `wall_robot_pkg` | Python | 节点 | 壁面机器人控制功能包 |
| `realsense-ros` | C++ | 驱动 | Intel RealSense 相机 ROS2 驱动（第三方） |
| `moveit2_calibration` | C++ | 插件 | MoveIt2 手眼标定功能（第三方） |
| `robot_move_ws` | C++ | 节点 | 通用机器人运动驱动工作空间 |

---

## 各功能包详细说明

### `agv_protocol` — 通信协议层

**路径**: `src/agv_protocol/`

**功能**: 提供 WebSocket 通信基础设施和 JSON 命令解析能力，是整个系统的通信基石。

**核心文件**:

| 文件 | 功能 |
|------|------|
| `include/agv_protocol/command_parser.hpp` | JSON 命令解析器头文件，定义消息结构体（Header、Command 等） |
| `include/agv_protocol/websocket_client.hpp` | WebSocket 客户端接口，支持异步连接、自动重连 |
| `include/agv_protocol/websocket_server.hpp` | WebSocket 服务端接口，支持多客户端连接 |
| `src/command_parser.cpp` | 命令解析实现：JSON 序列化/反序列化、消息验证 |
| `src/websocket_client.cpp` | WebSocket 客户端实现：基于 Boost.Beast 的 TLS 连接 |
| `src/websocket_server.cpp` | WebSocket 服务端实现：基于 Boost.Beast 的异步服务 |

**依赖**: `rclcpp`, `jsoncpp`, `boost`

**配置**: 无需额外配置，作为库被其他包链接使用。

---

### `agv_bridge` — WebSocket 桥接节点

**路径**: `src/agv_bridge/`

**功能**: 系统核心调度器，采用**中介者模式 + 组合模式**，在远程控制端与 ROS2 系统之间建立通信桥梁。

**核心文件**:

| 文件 | 功能 |
|------|------|
| `include/agv_bridge/device_handler.hpp` | 设备处理器抽象基类，定义 `init()` / `handleCommand()` / `getReport()` / `isOnline()` 接口 |
| `include/agv_bridge/robot.hpp` | 机器人抽象接口 `Robot` 和基础实现 `BaseRobot`，管理多个 DeviceHandler |
| `include/agv_bridge/handlers.hpp` | 内置设备处理器：`ChassisHandler`（底盘）、`LiftingHandler`（升降） |
| `include/agv_bridge/robots.hpp` | 具体机器人类 `AgvRobot` 和工厂 `AgvRobotFactory` |
| `include/agv_bridge/bridge_manager.hpp` | 桥接管理器：机器人管理、消息路由、状态上报、进程管理 |
| `src/bridge_manager.cpp` | 桥接管理器实现 |
| `src/websocket_bridge_node.cpp` | ROS2 节点入口，创建 WebSocket 客户端并连接远程服务器 |
| `AGV_BRIDGE_GUIDE.md` | 详细技术文档（含扩展指南） |

**ROS2 参数**:

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `server_ip` | string | `"127.0.0.1"` | WebSocket 服务器地址 |
| `server_port` | int | `8765` | WebSocket 服务器端口 |
| `report_interval` | double | `1.0` | 状态上报间隔（秒） |

**ROS2 话题**:

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 底盘速度控制 |
| `/odom` | `nav_msgs/Odometry` | 订阅 | 里程计数据 |
| `/battery_state` | `sensor_msgs/BatteryState` | 订阅 | 电池状态 |
| `/lifting_controller/command` | `std_msgs/Float64` | 发布 | 升降控制 |
| `/lifting_controller/state` | `std_msgs/Float64` | 订阅 | 升降状态 |

**启动方式**:
```bash
ros2 run agv_bridge websocket_bridge_node --ros-args -p server_ip:=192.168.1.100 -p server_port:=8765
```

**依赖**: `agv_protocol`, `rclcpp`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `Boost`, `OpenSSL`

---

### `agv_hardware` — 硬件接口层

**路径**: `src/agv_hardware/`

**功能**: 实现 `ros2_control` 的 `hardware_interface::SystemInterface`，对接珞石(Rokae)机械臂和 AGV 底盘硬件。

**核心文件**:

| 文件 | 功能 |
|------|------|
| `include/agv_hardware/agv_hardware.hpp` | 硬件接口核心类，实现 `on_init()` / `on_configure()` / `read()` / `write()` 等生命周期方法 |
| `include/agv_hardware/config.hpp` | 可配置参数：机械臂 IP、WebSocket 地址、初始化状态等 |
| `include/agv_hardware/agv_hardware_node.hpp` | 硬件节点头文件 |
| `src/agv_hardware_interface.cpp` | 硬件接口实现：设备初始化、状态读取、指令写入 |
| `src/agv_hardware_node.cpp` | 硬件节点主程序 |
| `src/robot_controller.cpp` | 珞石机械臂控制器封装：关节控制、笛卡尔控制、运动完成等待 |
| `include/function_helper.hpp` | 功能辅助函数 |
| `include/print_helper.hpp` | 打印辅助函数 |
| `include/rokae/` | 珞石机械臂 SDK 头文件 |
| `websocket_server.py` | Python WebSocket 服务端（测试用） |
| `AGVWebSocketServerFixed.cs` | C# WebSocket 服务端（Unity 端） |
| `READEME.md` | 硬件接口详细文档 |
| `ROS2.md` | 通信协议详细文档 |

**架构**:
```
┌─────────────────────────┐
│  AgvHardwareInterface   │  // 硬件接口核心
├─────────────────────────┤
│  ┌─────────────────────┐│
│  │  RobotController    ││  // 珞石机械臂控制
│  ├─────────────────────┤│
│  │  WebSocketClient    ││  // 向上位机发送关节数据
│  ├─────────────────────┤│
│  │  WebSocketServer    ││  // 接收上位机控制指令
│  └─────────────────────┘│
│  ┌─────────────────────┐│
│  │  CommandParser      ││  // JSON 命令解析
│  └─────────────────────┘│
└─────────────────────────┘
```

**配置**: 编辑 `include/agv_hardware/config.hpp` 中的 `AgvConfig` 结构体：
```cpp
robot_ip = "192.168.1.159";  // 珞石机械臂 IP
server_uri = "ws://192.168.1.100:8765";  // WebSocket 服务器地址
```

**依赖**: `rclcpp`, `rclcpp_lifecycle`, `hardware_interface`, `pluginlib`, `agv_protocol`, `agv_bridge`, `tf2_ros`, `jsoncpp`, `boost`

---

### `agv_description` — 机器人模型描述

**路径**: `src/agv_description/`

**功能**: 定义 AGV 机器人的完整 URDF 模型，包括底盘、双机械臂、升降机构、旋转机构的几何和运动学描述。

**核心文件**:

| 文件/目录 | 功能 |
|-----------|------|
| `urdf/agv_robot.urdf.xacro` | 主 URDF 文件，组合所有子模块 |
| `urdf/agv.xacro` | AGV 底盘 + 双臂 + 升降/旋转机构的 xacro 定义 |
| `urdf/agv_description.ros2_control.xacro` | ros2_control 硬件接口配置 |
| `meshes/base_link.STL` | AGV 底盘网格 |
| `meshes/lifting_link.STL` | 升降机构网格 |
| `meshes/revolute_link.STL` | 旋转机构网格 |
| `meshes/arm1_*.STL` | 左臂 6 关节网格（base, joint1~joint6） |
| `meshes/arm2_*.STL` | 右臂 6 关节网格（base, joint1~joint6） |
| `meshes/tool.STL` | 末端工具网格 |
| `config/joint_names_agv_description.yaml` | 关节名称配置 |
| `launch/display.launch.xml` | RViz 可视化 launch 文件 |
| `launch/gazebo.launch` | Gazebo 仿真 launch 文件 |
| `rviz/urdf_config.rviz` | RViz 配置文件 |

**关节结构**:
- `lifiting_joint` — 升降关节（棱柱副）
- `revolute_joint` — 旋转关节（旋转副）
- `arm1_joint1~6` — 左臂 6 自由度
- `arm2_joint1~6` — 右臂 6 自由度

**启动可视化**:
```bash
ros2 launch agv_description display.launch.xml
```

**依赖**: `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`

---

### `agv_bringup` — 系统启动配置

**路径**: `src/agv_bringup/`

**功能**: 系统级启动文件，一键启动完整的 AGV 控制系统（机器人描述、控制器、相机、MoveIt、RViz）。

**核心文件**:

| 文件 | 功能 |
|------|------|
| `launch/agv.launch.xml` | 主 launch 文件，启动所有组件 |
| `config/ros2_controllers.yaml` | ros2 控制器配置（关节轨迹控制器） |
| `config/agv_moveit.rviz` | RViz 可视化配置 |

**`agv.launch.xml` 启动内容**:
1. `robot_state_publisher` — 发布机器人 TF
2. `ros2_control_node` — 加载硬件接口
3. `joint_state_broadcaster` — 关节状态广播
4. `agvBase_controller` — 底盘（升降+旋转）轨迹控制器
5. `arm1_controller` — 左臂轨迹控制器
6. `arm2_controller` — 右臂轨迹控制器
7. `camera_with_tf.launch.py` — 相机启动 + TF 发布
8. `video_server` — 视频流 Web 服务
9. `move_group.launch.py` — MoveIt 运动规划
10. `rviz2` — 可视化界面

**控制器配置** (`ros2_controllers.yaml`):

| 控制器 | 类型 | 关节 |
|--------|------|------|
| `arm1_controller` | JointTrajectoryController | arm1_joint1~6 |
| `arm2_controller` | JointTrajectoryController | arm2_joint1~6 |
| `agvBase_controller` | JointTrajectoryController | lifiting_joint, revolute_joint |
| `joint_state_broadcaster` | JointStateBroadcaster | 所有关节 |

**启动方式**:
```bash
ros2 launch agv_bringup agv.launch.xml
```

**依赖**: `agv_description`, `agv_moveit_config`, `camera`, `controller_manager`, `robot_state_publisher`, `rviz2`

---

### `agv_moveit_config` — MoveIt 运动规划配置

**路径**: `src/agv_moveit_config/`

**功能**: 由 MoveIt Setup Assistant 自动生成的运动规划配置包，提供完整的运动规划能力。

**核心文件**:

| 文件 | 功能 |
|------|------|
| `config/agv_description.srdf` | 机器人语义描述（规划组、自碰撞、末端执行器） |
| `config/kinematics.yaml` | 运动学求解器配置 |
| `config/joint_limits.yaml` | 关节限位配置 |
| `config/moveit_controllers.yaml` | MoveIt 控制器配置 |
| `config/ros2_controllers.yaml` | ros2 控制器配置 |
| `config/initial_positions.yaml` | 初始关节位置 |
| `config/pilz_cartesian_limits.yaml` | Pilz 笛卡尔运动限位 |
| `launch/demo.launch.py` | 演示模式 launch |
| `launch/move_group.launch.py` | MoveIt 运动规划组 launch |
| `launch/moveit_rviz.launch.py` | MoveIt RViz 插件 launch |
| `launch/rsp.launch.py` | 机器人状态发布 launch |
| `launch/spawn_controllers.launch.py` | 控制器加载 launch |
| `launch/setup_assistant.launch.py` | Setup Assistant launch |

**启动演示**:
```bash
ros2 launch agv_moveit_config demo.launch.py
```

**依赖**: `moveit_ros_move_group`, `moveit_kinematics`, `moveit_planners`, `moveit_configs_utils`, `agv_description`, `controller_manager`, `rviz2`

---

### `camera` — 相机驱动与视觉

**路径**: `src/camera/`

**功能**: 奥视达(Astras)深度相机驱动，提供点云数据采集、TF 坐标发布和视频流 Web 服务。

**核心文件**:

| 文件 | 功能 |
|------|------|
| `include/camera/interface/camera_interface.hpp` | 相机抽象接口 |
| `include/camera/astras/astras_camera.hpp` | 奥视达相机驱动实现 |
| `include/camera/data/camera_types.hpp` | 相机数据类型定义 |
| `include/camera/nodes/astras_node.hpp` | 相机 ROS2 节点 |
| `src/astras/astras_camera.cpp` | 相机驱动实现 |
| `src/nodes/astras_node.cpp` | 相机节点实现 |
| `src/nodes/video_web_server.py` | 视频流 Web 服务器（Python） |
| `config/camera_tf.yaml` | 相机 TF 坐标配置 |
| `launch/camera_with_tf.launch.py` | 相机 + TF launch 文件 |
| `sdk/` | 奥视达相机 SDK（头文件和库） |

**ROS2 话题**:

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 深度点云数据 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |

**配置**: 编辑 `config/camera_tf.yaml` 设置相机安装位姿。

**启动方式**:
```bash
ros2 launch camera camera_with_tf.launch.py
```

**依赖**: `rclcpp`, `sensor_msgs`, `cv_bridge`, `image_transport`, `tf2_ros`

---

### `pathplannernode` — 路径规划节点

**路径**: `src/pathplannernode/`

**功能**: 基于点云数据的路径规划系统，支持注意力路径生成、局部坐标系转换和可视化。

**核心文件**:

| 文件 | 功能 |
|------|------|
| `src/pathplanner/pathplanner_ros/pathplanner_ros2_node.py` | ROS2 路径规划主节点 |
| `src/pathplanner/pathplanner/pointcloud_processing.py` | 点云数据处理（滤波、分割、特征提取） |
| `src/pathplanner/pathplanner/attention_path_ros2.py` | 注意力路径生成算法 |
| `src/pathplanner/pathplanner/local_coordinate.py` | 局部坐标系转换 |
| `src/pathplanner/pathplanner/visualization.py` | 路径可视化工具 |
| `src/pathplanner/setup.py` | Python 包安装配置 |
| `src/config/` | 路径规划配置文件 |
| `roi.png` | 感兴趣区域示例图 |
| `result/output.ply` | 点云输出结果示例 |

**依赖**: `rclpy`, `numpy`, `open3d`, `scipy`

---

### `wall_robot_pkg` — 壁面机器人功能包

**路径**: `src/wall_robot_pkg/`

**功能**: 壁面爬行机器人的 ROS2 Python 功能包。

**核心文件**:

| 文件 | 功能 |
|------|------|
| `wall_robot_pkg/` | Python 功能模块 |
| `launch/` | 启动文件 |
| `resource/` | 资源文件 |
| `test/` | 测试文件 |

**依赖**: `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`

---

### `realsense-ros` — Intel RealSense 相机驱动

**路径**: `src/realsense-ros/`

**功能**: Intel RealSense 深度相机的 ROS2 官方驱动（第三方包），支持 D400 系列和 L500 系列相机。

**子包**:
- `realsense2_camera` — 相机驱动主包
- `realsense2_camera_msgs` — 自定义消息和服务
- `realsense2_description` — 相机 URDF 模型

**来源**: [Intel RealSense ROS](https://github.com/IntelRealSense/realsense-ros)

---

### `moveit2_calibration` — 手眼标定

**路径**: `src/moveit2_calibration/`

**功能**: MoveIt2 手眼标定功能（第三方包），提供 RViz 插件和标定求解器。

**子包**:
- `moveit_calibration_demos` — 标定演示
- `moveit_calibration_gui` — RViz 标定 GUI 插件
- `moveit_calibration_plugins` — 标定求解器插件

**来源**: [MoveIt Calibration](https://github.com/moveit/moveit2_calibration)

---

### `robot_move_ws` — 机器人运动工作空间

**路径**: `src/robot_move_ws/`

**功能**: 通用机器人运动驱动工作空间，包含机器人运动控制的底层实现。

**核心文件**:
- `src/robot_go_target/src/universal_robot_driver.cpp` — 通用机器人驱动，支持目标点导航

---

## 通信协议

### 消息格式

所有 WebSocket 消息采用统一 JSON 格式：

```json
{
    "header": {
        "robot_id": "mobile_dual_arm_robot",
        "msg_type": "command"
    },
    "payload": {
        // 具体指令内容
    }
}
```

### 消息类型 (msg_type)

| 类型 | 方向 | 说明 |
|------|------|------|
| `check` | 双向 | 设备使能/自检 |
| `command` | 上位机→下位机 | 控制指令 |
| `response` | 下位机→上位机 | 指令响应 |
| `status_update` | 下位机→上位机 | 周期性状态上报 |

### 支持的机器人类型 (robot_id)

| robot_id | 说明 |
|----------|------|
| `vacuum_adsorption_robot` | 气吸附机器人 |
| `Magnetic Adsorption Robot` | 磁吸附机器人 |
| `mobile_dual_arm_robot` | AGV + 双臂机器人 |

### 控制指令示例

**底盘运动**:
```json
{
    "header": { "robot_id": "agv", "msg_type": "command" },
    "payload": {
        "agv": {
            "command": "move",
            "data": { "vx": 0.5, "vy": 0.0, "wz": 0.0 }
        }
    }
}
```

**机械臂关节控制**:
```json
{
    "header": { "robot_id": "mobile_dual_arm_robot", "msg_type": "command" },
    "payload": {
        "arms": [
            {
                "arm_id": "left_arm",
                "command": "joint_move",
                "parameters": {
                    "val1": 0.2, "val2": 0.1, "val3": 0.3,
                    "val4": 0.0, "val5": 0.5, "val6": 0.0,
                    "speed": 45
                }
            }
        ]
    }
}
```

**机械臂笛卡尔控制**:
```json
{
    "header": { "robot_id": "mobile_dual_arm_robot", "msg_type": "command" },
    "payload": {
        "arms": [
            {
                "arm_id": "left_arm",
                "command": "cartesian_move",
                "parameters": {
                    "x": 0.2, "y": 0.1, "z": 0.3,
                    "roll": 0.0, "pitch": 0.5, "yaw": 0.0,
                    "speed": 45
                }
            }
        ]
    }
}
```

**急停**:
```json
{
    "header": { "robot_id": "agv", "msg_type": "command" },
    "payload": {
        "agv": { "command": "emergency_stop" }
    }
}
```

> 更多协议细节请参考 `src/agv_hardware/ROS2.md` 和 `src/agv_bridge/AGV_BRIDGE_GUIDE.md`。

---

## 环境要求与依赖安装

### 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS 2 版本**: Humble Hawksbill
- **编译工具**: colcon, CMake >= 3.8

### 安装 ROS 2 Humble

```bash
# 添加 ROS 2 apt 源
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装开发工具
sudo apt install python3-colcon-common-extensions python3-rosdep
```

### 安装项目依赖

```bash
# 安装 MoveIt 2
sudo apt install ros-humble-moveit

# 安装 ros2_control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# 安装其他依赖
sudo apt install \
    libboost-all-dev \
    libssl-dev \
    libjsoncpp-dev \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui

# Python 依赖
pip3 install open3d numpy scipy
```

---

## 编译与运行

### 编译

```bash
# 克隆仓库
git clone https://github.com/Dtsyyds/AGV_ROS2_Control_Manager.git
cd AGV_ROS2_Control_Manager

# source ROS 2
source /opt/ros/humble/setup.bash

# 编译（跳过第三方包以加速）
colcon build --symlink-install --packages-skip realsense-ros moveit2_calibration

# 或编译全部
colcon build --symlink-install
```

### 运行

```bash
# source 工作空间
source install/setup.bash

# 方式 1：启动完整系统
ros2 launch agv_bringup agv.launch.xml

# 方式 2：单独启动各组件
# 启动 WebSocket 桥接节点
ros2 run agv_bridge websocket_bridge_node --ros-args \
    -p server_ip:=192.168.1.100 \
    -p server_port:=8765 \
    -p report_interval:=1.0

# 启动 MoveIt 演示
ros2 launch agv_moveit_config demo.launch.py

# 启动相机
ros2 launch camera camera_with_tf.launch.py

# 启动路径规划节点
ros2 run pathplannernode pathplanner_ros2_node
```

---

## 如何上传到 GitHub

### 前提条件

1. 拥有 GitHub 账号
2. 已安装 Git
3. 已配置 SSH 密钥或 HTTPS 凭据

### 步骤

#### 1. 配置 Git 用户信息（如未配置）

```bash
git config --global user.name "你的用户名"
git config --global user.email "你的邮箱"
```

#### 2. 在 GitHub 上创建仓库

1. 访问 [https://github.com/new](https://github.com/new)
2. 填写仓库名，如 `AGV_ROS2_Control_Manager`
3. 选择 Public 或 Private
4. **不要**勾选 "Add a README file"（本地已有）
5. 点击 "Create repository"

#### 3. 初始化本地仓库并推送

```bash
# 进入项目目录
cd /home/dts/agv_git

# 初始化 Git 仓库（如果尚未初始化）
git init

# 添加远程仓库
git remote add origin https://github.com/你的用户名/AGV_ROS2_Control_Manager.git

# 添加所有文件到暂存区
git add .

# 查看将要提交的文件（确认没有不需要的文件）
git status

# 提交
git commit -m "feat: 初始提交 - AGV ROS2 多机器人控制平台"

# 推送到 GitHub
git push -u origin main
```

#### 4. 后续更新推送

```bash
# 修改文件后
git add .
git commit -m "描述你的修改"
git push
```

### 注意事项

- `.gitignore` 已配置排除 `build/`、`install/`、`log/`、`*.log`、`*.tmp`、`*.pt` 等编译产物和临时文件
- `realsense-ros` 和 `moveit2_calibration` 是第三方包，建议使用 `git submodule` 管理，或在 README 中说明安装方式
- 大型二进制文件（如 `.STL` 网格、SDK 库）建议使用 [Git LFS](https://git-lfs.github.com/) 管理

---

## 项目结构

```
AGV_ROS2_Control_Manager/
├── README.md                          # 本文件
├── .gitignore                         # Git 忽略规则
├── ROS2_Servernode.md                 # Server Node 通信说明
│
└── src/
    ├── agv_protocol/                  # 📡 通信协议库
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/agv_protocol/
    │   │   ├── command_parser.hpp     # JSON 命令解析器
    │   │   ├── websocket_client.hpp   # WebSocket 客户端
    │   │   └── websocket_server.hpp   # WebSocket 服务端
    │   └── src/
    │       ├── command_parser.cpp
    │       ├── websocket_client.cpp
    │       └── websocket_server.cpp
    │
    ├── agv_bridge/                    # 🌉 WebSocket 桥接节点
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── AGV_BRIDGE_GUIDE.md        # 详细技术文档
    │   ├── include/agv_bridge/
    │   │   ├── device_handler.hpp     # 设备处理器接口
    │   │   ├── robot.hpp              # 机器人抽象基类
    │   │   ├── handlers.hpp           # 底盘/升降处理器
    │   │   ├── robots.hpp             # 具体机器人工厂
    │   │   └── bridge_manager.hpp     # 桥接管理器
    │   └── src/
    │       ├── bridge_manager.cpp
    │       └── websocket_bridge_node.cpp
    │
    ├── agv_hardware/                  # 🔧 硬件接口层
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── agv_hardware_interface.xml # 插件描述文件
    │   ├── READEME.md                 # 硬件接口文档
    │   ├── ROS2.md                    # 通信协议文档
    │   ├── websocket_server.py        # Python WebSocket 测试服务端
    │   ├── AGVWebSocketServerFixed.cs # Unity C# WebSocket 服务端
    │   ├── include/
    │   │   ├── agv_hardware/
    │   │   │   ├── agv_hardware.hpp   # 硬件接口核心
    │   │   │   ├── config.hpp         # 配置参数
    │   │   │   └── agv_hardware_node.hpp
    │   │   ├── rokae/                 # 珞石机械臂 SDK
    │   │   ├── function_helper.hpp
    │   │   └── print_helper.hpp
    │   ├── src/
    │   │   ├── agv_hardware_interface.cpp
    │   │   ├── agv_hardware_node.cpp
    │   │   └── robot_controller.cpp
    │   └── external/Eigen/            # Eigen 线性代数库
    │
    ├── agv_description/               # 🤖 机器人 URDF 模型
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── urdf/
    │   │   ├── agv_robot.urdf.xacro   # 主 URDF 文件
    │   │   ├── agv.xacro              # AGV + 双臂定义
    │   │   └── agv_description.ros2_control.xacro
    │   ├── meshes/                    # STL 网格文件
    │   │   ├── base_link.STL
    │   │   ├── lifting_link.STL
    │   │   ├── revolute_link.STL
    │   │   ├── arm1_*.STL             # 左臂各关节
    │   │   ├── arm2_*.STL             # 右臂各关节
    │   │   └── tool.STL
    │   ├── config/
    │   │   └── joint_names_agv_description.yaml
    │   ├── launch/
    │   │   ├── display.launch.xml     # RViz 可视化
    │   │   └── gazebo.launch          # Gazebo 仿真
    │   └── rviz/urdf_config.rviz
    │
    ├── agv_bringup/                   # 🚀 系统启动配置
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/agv.launch.xml      # 主 launch 文件
    │   └── config/
    │       ├── ros2_controllers.yaml  # 控制器配置
    │       └── agv_moveit.rviz        # RViz 配置
    │
    ├── agv_moveit_config/             # 🎯 MoveIt 运动规划
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── .setup_assistant
    │   ├── config/
    │   │   ├── agv_description.srdf   # 语义描述
    │   │   ├── kinematics.yaml        # 运动学配置
    │   │   ├── joint_limits.yaml      # 关节限位
    │   │   ├── moveit_controllers.yaml
    │   │   ├── ros2_controllers.yaml
    │   │   ├── initial_positions.yaml
    │   │   └── pilz_cartesian_limits.yaml
    │   └── launch/
    │       ├── demo.launch.py
    │       ├── move_group.launch.py
    │       ├── moveit_rviz.launch.py
    │       ├── rsp.launch.py
    │       ├── spawn_controllers.launch.py
    │       └── setup_assistant.launch.py
    │
    ├── camera/                        # 📷 相机驱动
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── README.md
    │   ├── config/camera_tf.yaml      # 相机 TF 配置
    │   ├── launch/camera_with_tf.launch.py
    │   ├── include/camera/
    │   │   ├── interface/camera_interface.hpp
    │   │   ├── astras/astras_camera.hpp
    │   │   ├── data/camera_types.hpp
    │   │   └── nodes/astras_node.hpp
    │   ├── src/
    │   │   ├── astras/astras_camera.cpp
    │   │   └── nodes/
    │   │       ├── astras_node.cpp
    │   │       └── video_web_server.py
    │   └── sdk/                       # 奥视达相机 SDK
    │
    ├── pathplannernode/               # 🗺️ 路径规划
    │   ├── roi.png
    │   ├── result/output.ply
    │   └── src/
    │       ├── config/
    │       └── pathplanner/
    │           ├── setup.py
    │           ├── pathplanner/
    │           │   ├── pointcloud_processing.py
    │           │   ├── attention_path_ros2.py
    │           │   ├── local_coordinate.py
    │           │   └── visualization.py
    │           └── pathplanner_ros/
    │               └── pathplanner_ros2_node.py
    │
    ├── wall_robot_pkg/                # 🧱 壁面机器人
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── launch/
    │   ├── wall_robot_pkg/
    │   ├── resource/
    │   └── test/
    │
    ├── realsense-ros/                 # 📸 Intel RealSense 驱动 (第三方)
    ├── moveit2_calibration/           # 📐 手眼标定 (第三方)
    └── robot_move_ws/                 # 🤖 通用机器人运动驱动
```

---

## 许可证

- 主要功能包: Apache License 2.0
- MoveIt 配置: BSD-3-Clause
- 第三方包遵循各自许可证

## 维护者

- **dts** — 主要开发者
- **zhuchen** — 硬件接口与 MoveIt 配置
- **zyj** — 相机驱动