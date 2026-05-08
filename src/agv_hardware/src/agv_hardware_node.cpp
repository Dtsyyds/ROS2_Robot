#include "agv_hardware/agv_hardware_node.hpp"

namespace agv_hardware {

AgvHardwareNode::AgvHardwareNode(const rclcpp::NodeOptions & options)
: Node("agv_hardware_topic_node", options)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void AgvHardwareNode::init(RobotController* controller)
{
    robot_controller_ = controller;

    // 1. 状态发布者
    joint_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("arm_joint_states", 10);
    cartesian_pose_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("arm_cartesian_pose", 10);
    click_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/click_point", 10);

    // 2. 关节指令订阅者
    joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "cmd_arm_joint", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (!robot_controller_) return;
            if (msg->data.empty() || msg->data.size() % 6 != 0) {
                RCLCPP_WARN(this->get_logger(), "忽略关节指令：数据长度 %zu 不是 6 的整数倍", msg->data.size());
                return;
            }

            std::vector<std::array<double, 6>> points;
            points.reserve(msg->data.size() / 6);
            for (size_t offset = 0; offset < msg->data.size(); offset += 6) {
                std::array<double, 6> target;
                std::copy(msg->data.begin() + offset, msg->data.begin() + offset + 6, target.begin());
                points.push_back(target);
            }

            RCLCPP_INFO(this->get_logger(), "收到关节指令，点数: %zu，正在下发...", points.size());
            if (points.size() == 1) {
                robot_controller_->sendJointCommand(points.front());
            } else {
                robot_controller_->sendCartesianCommand(points, "joint", 100.0);
            }
        });

    // 3. 笛卡尔指令订阅者
    cartesian_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "cmd_arm_cartesian", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (!robot_controller_) return;
            if (msg->data.empty() || msg->data.size() % 6 != 0) {
                RCLCPP_WARN(this->get_logger(), "忽略笛卡尔指令：数据长度 %zu 不是 6 的整数倍", msg->data.size());
                return;
            }

            std::vector<std::array<double, 6>> points;
            points.reserve(msg->data.size() / 6);
            for (size_t offset = 0; offset < msg->data.size(); offset += 6) {
                std::array<double, 6> p;
                std::copy(msg->data.begin() + offset, msg->data.begin() + offset + 6, p.begin());
                points.push_back(p);
            }

            RCLCPP_INFO(this->get_logger(), "收到笛卡尔指令，点数: %zu，正在下发...", points.size());
            robot_controller_->sendCartesianCommand(points, "linear", 100.0);
        });

    // 4. 轨迹路径订阅者 (PoseArray)
    cartesian_path_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "/path_planner/cartesian_path", 10,
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            if (msg->poses.empty()) return;

            std::cerr << "收到路径规划轨迹，点数: " << msg->poses.size() << " (frame: " << msg->header.frame_id << ")" << std::endl;

            std::vector<std::array<double, 6>> cartesian_path;
            for (const auto& pose : msg->poses) {
                // 直接使用位置（米）和四元数，转换为欧拉角
                tf2::Quaternion q(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w);
                    // tf2::Quaternion q(
                    // 0.13497,
                    // 0.75322,
                    // -0.63979,
                    // -0.071439);
                double rx, ry, rz;
                tf2::Matrix3x3(q).getRPY(rx, ry, rz);

                std::array<double, 6> p = {
                    pose.position.x,
                    pose.position.y,
                    pose.position.z ,
                    rx, ry, rz
                };
                cartesian_path.push_back(p);
            }

            if (!cartesian_path.empty()) {
                RCLCPP_INFO(this->get_logger(), "下发整条规划轨迹，共 %zu 个点", cartesian_path.size());
                for(size_t i=0; i<cartesian_path.size(); i++){
                    robot_controller_->sendCartesianCommand({cartesian_path[i]}, "linear", 100.0);
                    robot_controller_->waitForMotionComplete(5000);
                }
            }
        });
}

void AgvHardwareNode::publishJointState(const std::array<double, 6>& positions)
{
    if (joint_state_pub_) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.assign(positions.begin(), positions.end());
        joint_state_pub_->publish(msg);
    }
}

void AgvHardwareNode::publishCartesianPose(const std::array<double, 6>& posture)
{
    if (cartesian_pose_pub_) {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.assign(posture.begin(), posture.end());
        cartesian_pose_pub_->publish(msg);
    }
}

} // namespace agv_hardware
