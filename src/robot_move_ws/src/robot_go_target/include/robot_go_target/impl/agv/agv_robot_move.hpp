#ifndef AGV_ROBOT_MOVE_HPP
#define AGV_ROBOT_MOVE_HPP

#include "robot_go_target/interface/robot_move_base.hpp"
#include <string>
#include <vector>
#include <cstdint>

// 引入 TF2 线性数学库核心声明
#include <tf2/LinearMath/Transform.h>
// 引入 ROS 2 日志声明
#include <rclcpp/rclcpp.hpp> 

class AGVRobotMove : public RobotMove, public IAdvancedNavigation, public IPoseProvider
{
private:
    int sock_19204_ = -1, sock_19205_ = -1, sock_19206_ = -1, sock_19210_ = -1;
    uint16_t seq_num_ = 0;

    // ==========================================
    // TF2 变换矩阵，用于记录启动原点
    // ==========================================
    tf2::Transform start_transform_;
    bool has_started_ = false;
    
    // 内部私有方法声明
    rclcpp::Logger get_logger();
    bool connectPort(const std::string& ip, int port, int& sock_fd);
    std::vector<uint8_t> packSeerFrame(uint16_t type, const std::string& json);
    void unlockBrake();
    void readresponse(int sock_fd, const std::string& cmd_name);
    bool getcurrentpose(double& x, double& y, double& theta);
    void robot_control_reloc_req();
    bool robot_navigation_status();

public:
    AGVRobotMove();
    ~AGVRobotMove() override = default;

    // 重写接口的公开方法声明
    bool init(const std::string& seer_ip) override;
    bool getpose(double& x, double& y, double& theta) override;
    void setVelocity(double vx, double vy, double wz) override;
    bool movebydistance(double dist, double vx, double vy) override;
    bool rotatebyangle(double angle, double vw) override;
    void send_loop() override;
    void moveshutdown() override;
};

#endif // AGV_ROBOT_MOVE_HPP