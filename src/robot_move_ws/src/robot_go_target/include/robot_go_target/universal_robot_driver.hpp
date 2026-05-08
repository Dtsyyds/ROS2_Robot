#ifndef UNIVERSAL_ROBOT_DRIVER_HPP
#define UNIVERSAL_ROBOT_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/empty.hpp> 
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// 引入硬件基类
#include "robot_go_target/interface/robot_move_base.hpp"

#include <memory>
#include <mutex>
#include <string>

class UniversalRobotDriver : public rclcpp::Node
{
private:
    std::shared_ptr<RobotMove> robot_;
    
    // 订阅者
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr robot_move_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_distance_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_rotate_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr scan_config_sub_; // 扫查器配置
    // 声明一个订阅者，用于接收界面的扫查指令 (True/False)
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scan_cmd_sub_;

    // 位置发布者
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    //声明一个定时器，用于高频运行扫查状态机
    rclcpp::TimerBase::SharedPtr state_machine_timer_;


    // --- 线程相关 ---
    std::mutex robot_mutex_; // 保护 robot_ 指针的互斥锁
    rclcpp::Time last_cmd_time_; // 看门狗时间戳

    // 缓存当前速度指令
    double target_linear_x_ = 0.0;
    double target_linear_y_ = 0.0;
    double target_angular_z_ = 0.0;

    // 回调函数声明
    void cmdVelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void scanconfigCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // 扫查指令回调函数声明
    void scan_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void stepMoveCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void stepRotateCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timerCallback();
    // 定时器回调函数声明
    void state_machine_callback();

public:
    UniversalRobotDriver(const std::string &node_name);
    ~UniversalRobotDriver();
};

#endif // UNIVERSAL_ROBOT_DRIVER_HPP