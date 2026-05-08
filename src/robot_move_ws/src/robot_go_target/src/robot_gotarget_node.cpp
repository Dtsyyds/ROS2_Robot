// ==========================================
// 节点启动文件
// ==========================================
#include <rclcpp/rclcpp.hpp>

// 引入刚刚拆分出去的节点头文件
#include "robot_go_target/universal_robot_driver.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 实例化刚刚封装好的独立驱动节点
    auto node = std::make_shared<UniversalRobotDriver>("robot_gotarget_node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}