# agv_description

## 🤖 功能描述
`agv_description` 包含了 AGV 机器人的完整物理描述模型（URDF/xacro）。它定义了机器人的几何形状、运动学参数、视觉网格（STL）以及 `ros2_control` 的硬件配置接口。

## 🏗️ 机器人结构
- **底盘 (Base)**: 移动平台基础。
- **升降机构 (Lifting)**: 支持垂直升降运动。
- **旋转机构 (Revolute)**: 支持水平旋转运动。
- **双机械臂 (Dual Arms)**: 搭载两台珞石 6 轴机械臂。
- **末端工具 (Tool)**: 机械臂末端的夹持器或工具。

## 📂 核心资源
- `urdf/`: 机器人描述文件
- `meshes/`: STL 视觉与碰撞模型
- `rviz/`: 预设的可视化配置文件
- `launch/`: 模型预览启动文件

## 🚀 启动预览
要在 RViz 中查看机器人模型：
```bash
ros2 launch agv_description display.launch.xml
```

---
*更多详细说明请参考项目根目录下的 [README.md](../../README.md)。*
