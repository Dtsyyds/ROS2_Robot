#include "robot_go_target/factory/robot_factory.hpp"

#include "robot_go_target/impl/agv/agv_robot_move.hpp"
#include "robot_go_target/impl/air/air_robot_move.hpp"
#include "robot_go_target/impl/duct/duct_robot_move.hpp"
#include "robot_go_target/impl/mag/mag_robot_move.hpp"

namespace robot_go_target {

std::shared_ptr<RobotMove> createRobot(const std::string& type, const std::string& port, rclcpp::Logger logger)
{
    std::shared_ptr<RobotMove> robot;

    if (type == "air") 
    {
        robot = std::make_shared<AirRobotMove>();
    } 
    else if (type == "agv") 
    {
        robot = std::make_shared<AGVRobotMove>();
    } 
    else if (type == "mag") 
    {
        robot = std::make_shared<MagRobotMove>();
    } 
    else if (type == "duct") 
    {
        robot = std::make_shared<DuctRobotMove>();
    } 
    else 
    {
        RCLCPP_ERROR(logger, "未知机器人类型: %s", type.c_str());
        return nullptr;
    }

    // 初始化硬件连接
    if (!robot->init(port)) 
    {
        RCLCPP_FATAL(logger, "硬件初始化失败! Port: %s", port.c_str());
        return nullptr;
    }

    RCLCPP_INFO(logger, "[RobotFactory] 硬件实例化与初始化成功!");
    return robot;
}

} // namespace robot_go_target