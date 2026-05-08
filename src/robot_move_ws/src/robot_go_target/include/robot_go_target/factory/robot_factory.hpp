#ifndef ROBOT_FACTORY_HPP
#define ROBOT_FACTORY_HPP

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "robot_go_target/interface/robot_move_base.hpp"

namespace robot_go_target {

// 根据类型和端口，实例化并初始化机器人 (注意：必须使用 shared_ptr 保证多线程安全)
std::shared_ptr<RobotMove> createRobot(const std::string& type, const std::string& port, rclcpp::Logger logger);

} // namespace robot_go_target

#endif // ROBOT_FACTORY_HPP