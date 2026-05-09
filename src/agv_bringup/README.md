# agv_bringup

## 🚀 功能描述
`agv_bringup` 是整个系统的集成启动包。它负责加载参数、启动核心节点、加载控制器以及启动可视化界面，实现系统的“一键启动”。

## 🛠️ 主要内容
- **Launch 文件**: `agv.launch.xml` 整合了底盘、机械臂、相机和运动规划的启动逻辑。
- **控制器配置**: `ros2_controllers.yaml` 定义了所有关节的硬件控制器参数。
- **可视化**: 预配置了 `rviz2` 界面。

## ⚙️ 启动命令
启动完整的 AGV 控制系统：
```bash
ros2 launch agv_bringup agv.launch.xml
```

---
*更多详细说明请参考项目根目录下的 [README.md](../../README.md)。*
