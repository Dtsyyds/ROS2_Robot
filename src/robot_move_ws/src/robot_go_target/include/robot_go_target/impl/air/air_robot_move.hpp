#ifndef AIR_ROBOT_MOVE_HPP
#define AIR_ROBOT_MOVE_HPP

#include "robot_go_target/interface/robot_move_base.hpp"
#include <vector>
#include <string>
#include <cstdint>

// 引入 ROS 2 核心库以使用日志系统
#include <rclcpp/rclcpp.hpp>

class AirRobotMove : public RobotMove
{
private:
    int serial_fd_ = -1;
    std::vector<uint16_t> ch_;

    // 内部私有方法声明
    rclcpp::Logger get_logger();
    void pack_protocol_data(std::vector<uint16_t> ch, uint8_t* buf);

public:
    AirRobotMove();
    ~AirRobotMove() override = default;

    // 重写接口的公开方法声明
    bool init(const std::string& port) override;
    void setVelocity(double linear_x, double vy, double angle_z) override;
    void send_loop() override;
    void moveshutdown() override;

    // 常量定义 (C++11 支持在头文件中直接初始化非静态常量成员)
    const std::vector<uint16_t> BACKWARD   = {1500,1730,1500,1500,1,1,1,0, 0,0,1050,1950,0,0,0,0};
    const std::vector<uint16_t> FORWARD    = {1500,1270,1500,1500,1,1,1,0, 0,0,1050,1950,0,0,0,0};
    const std::vector<uint16_t> TURN_LEFT  = {1200,1500,1500,1500,1,1500,1,1500, 1500,1500,1050,1950,0,0,0,0};
    const std::vector<uint16_t> TURN_RIGHT = {1900,1500,1500,1500,1,1500,1,1500, 1500,1500,1050,1950,0,0,0,0};
    const std::vector<uint16_t> STOP       = {1500,1500,1500,1500,1,1,1,0, 0,0,1050,1900,0,0,0,0};
};

#endif // AIR_ROBOT_MOVE_HPP