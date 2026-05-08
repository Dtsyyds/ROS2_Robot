import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros

def generate_launch_description():
    # 1、声明一个launch参数
    action_robot_type_arg = launch.actions.DeclareLaunchArgument(
        'type', #参数名
        default_value="agv", #默认值
        description="Robot type: air, agv, mag, duct"
    )
    
    # 2、获取命令行传入的机型参数
    robot_type_val = launch.substitutions.LaunchConfiguration('type')
    
    # ========================================================
    # 3、获取 YAML 配置文件的绝对路径
    # 这要求你的 yaml 文件放在 src/robot_go_target/config/robot_config.yaml
    # ========================================================
    config_dir = os.path.join(get_package_share_directory('robot_go_target'), 'config')
    config_file = os.path.join(config_dir, 'robot_config.yaml')

    # 4、启动节点并传递参数
    action_node_robot_move_node  = launch_ros.actions.Node(
        package='robot_go_target' ,
        executable='robot_gotarget_node',
        name='robot_gotarget_node',     # 指定节点名称，必须与 YAML 文件顶部的名称一致
        output='screen',
        parameters=[
            config_file,                    # 加载 yaml 配置文件中的底层硬件端口参数
            {'robot_type': robot_type_val}  # 动态加载命令行传入的机型参数
        ]
    )
    
    return launch.LaunchDescription([
        action_robot_type_arg,
        action_node_robot_move_node
    ])