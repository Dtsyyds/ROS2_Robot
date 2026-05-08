#include "agv_hardware/agv_hardware.hpp"
#include "agv_hardware/self_check_manager.hpp"
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace agv_hardware {

hardware_interface::CallbackReturn AgvHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    config_ = AgvConfig();
    robot_controller_ = std::make_unique<RobotController>(config_);
    
    // ==================== 初始化 ROS 2 话题接口 ====================
    topic_node_ = std::make_shared<AgvHardwareNode>();
    topic_node_->init(robot_controller_.get());
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AgvHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return robot_controller_->connect() ? 
        hardware_interface::CallbackReturn::SUCCESS : 
        hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn AgvHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (!robot_controller_->activate()) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    joint_positions_ = robot_controller_->getJointPositions();

    // 启动独立线程处理 ROS 2 话题回调
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(topic_node_);
    executor_thread_ = std::make_unique<std::thread>([this]() {
        executor_->spin();
    });

    // ==================== 系统初始化完成，移动到初始位姿 ====================
    // 角度值：60, -90, -120, -150, -108, 10
    #ifndef M_PI
    #define M_PI 3.14159265358979323846
    #endif
    const double deg2rad = M_PI / 180.0;
    std::array<double, 6> init_pose = {
        60.0 * deg2rad,
        -90.0 * deg2rad,
        -120.0 * deg2rad,
        -150.0 * deg2rad,
        -108.0 * deg2rad,
        10.0 * deg2rad
    };

    RCLCPP_INFO(topic_node_->get_logger(), "系统初始化完成，机械臂正在运行至初始位姿...");
    robot_controller_->sendJointCommand(init_pose);
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AgvHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    // 停止 Executor 线程
    if (executor_) {
        executor_->cancel();
    }
    if (executor_thread_ && executor_thread_->joinable()) {
        executor_thread_->join();
    }

    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        enable_status_report_ = false;
    }
    robot_controller_->deactivate();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AgvHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // 1. 更新硬件状态
    {
        std::lock_guard<std::mutex> lock(io_mutex_);
        joint_positions_ = robot_controller_->getJointPositions();
        posture_positions_ = robot_controller_->getPosturePositions();
    }
    
    // 3. 通过重构后的节点发布状态
    if (topic_node_) {
        topic_node_->publishJointState(joint_positions_);
        topic_node_->publishCartesianPose(posture_positions_);
    }

    set_state("arm2_joint1/position", joint_positions_[0]);
    set_state("arm2_joint2/position", joint_positions_[1]);
    set_state("arm2_joint3/position", joint_positions_[2]);
    set_state("arm2_joint4/position", joint_positions_[3]);
    set_state("arm2_joint5/position", joint_positions_[4]);
    set_state("arm2_joint6/position", joint_positions_[5]);
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgvHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // 实时控制模式
    if (config_.rt_mode) {
        std::lock_guard<std::mutex> lock(io_mutex_);
        robot_controller_->sendJointCommand(joint_commands_);
    }
    return hardware_interface::return_type::OK;
}


} // namespace agv_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    agv_hardware::AgvHardwareInterface,
    hardware_interface::SystemInterface)
