#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <memory>
#include <mutex>
#include <array>
#include <vector>
#include <string>
#include "agv_hardware/robot_controller.hpp"

namespace agv_hardware {

/**
 * @brief 专门处理 ROS 2 话题通信的类
 * 
 * 将原本在 AgvHardwareInterface 中的订阅和发布逻辑剥离出来，
 * 保持硬件接口类的纯净。
 */
class AgvHardwareNode : public rclcpp::Node {
public:
    explicit AgvHardwareNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    /**
     * @brief 初始化订阅者和发布者
     * @param controller 指向机器人控制器的指针，用于下发指令
     */
    void init(RobotController* controller);

    /**
     * @brief 发布当前关节状态
     * @param positions 关节位置
     */
    void publishJointState(const std::array<double, 6>& positions);

    /**
     * @brief 发布当前笛卡尔位姿
     * @param posture 笛卡尔位姿
     */
    void publishCartesianPose(const std::array<double, 6>& posture);

private:
    RobotController* robot_controller_ = nullptr;
    std::mutex* io_mutex_ptr_ = nullptr; // 可选：如果需要共用锁

    // 发布者
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr click_point_pub_;

    // 订阅者
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cartesian_path_sub_;

    // TF 相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace agv_hardware
