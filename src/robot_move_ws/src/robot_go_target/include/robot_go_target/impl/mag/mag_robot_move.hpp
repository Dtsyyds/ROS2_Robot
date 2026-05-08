#ifndef MAG_ROBOT_MOVE_HPP
#define MAG_ROBOT_MOVE_HPP

#include "robot_go_target/interface/robot_move_base.hpp"
#include <vector>
#include <string>
#include <cstdint>

// 引入 ROS 2 核心库以使用日志系统
#include <rclcpp/rclcpp.hpp>

class MagRobotMove : public RobotMove
{
private:
    int serial_fd_ = -1;
    std::vector<uint16_t> ch_; 

    // 内部私有方法声明
    rclcpp::Logger get_logger();
    void pack_protocol_data(const std::vector<uint16_t>& ch, uint8_t* buf);

    // 常量定义
    const uint16_t VAL_LOW  = 362;
    const uint16_t VAL_MID  = 1002;
    const uint16_t VAL_HIGH = 1642;

public:
    MagRobotMove();
    ~MagRobotMove() override = default;

    // 重写接口的公开方法声明
    bool init(const std::string& port) override;
    void setVelocity(double linear_x, double vy, double angle_z) override;
    void send_loop() override;
    void moveshutdown() override;
};

#endif // MAG_ROBOT_MOVE_HPP