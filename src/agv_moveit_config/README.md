# agv_moveit_config

## 🎯 功能描述
`agv_moveit_config` 提供了基于 **MoveIt 2** 的运动规划配置。它定义了机器人的规划组（左臂、右臂、底盘机构）、运动学求解器参数以及避障规则。

## 🛠️ 核心配置
- **规划组**: `arm1`, `arm2`, `agv_base` 等。
- **SRDF**: 定义了自碰撞矩阵和虚拟关节。
- **轨迹控制**: 与 `ros2_control` 协同工作的控制器配置。

## 🚀 启动示例
启动 MoveIt 演示模式（包含 RViz）：
```bash
ros2 launch agv_moveit_config demo.launch.py
```

---
*更多详细说明请参考项目根目录下的 [README.md](../../README.md)。*
