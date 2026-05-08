#include "robot_go_target/universal_robot_driver.hpp"

// 引入工厂类用于实例化
#include "robot_go_target/factory/robot_factory.hpp" 

#include <thread>
#include <functional>
#include <chrono>

UniversalRobotDriver::UniversalRobotDriver(const std::string &node_name) : Node(node_name)
{
    RCLCPP_INFO(this->get_logger(),"[%s]节点启动!", node_name.c_str());
    
    // 1. 声明并获取机型参数 "robot_type"
    this->declare_parameter<std::string>("robot_type", "agv");
    std::string type = this->get_parameter("robot_type").as_string();
    
    // 2. 声明所有的端口参数（默认值仅作为 fallback）
    this->declare_parameter<std::string>("ports.air", "/dev/ttyCH341USB0");
    this->declare_parameter<std::string>("ports.agv", "10.42.0.114");
    this->declare_parameter<std::string>("ports.mag", "/dev/ttyUSB1");
    this->declare_parameter<std::string>("ports.duct", "/dev/ttyACM0");

    // 3. 动态拼接需要获取的参数名，例如 "ports.agv"
    std::string param_name = "ports." + type;
    std::string port = this->get_parameter(param_name).as_string();
    
    RCLCPP_INFO(this->get_logger(), "从 YAML 读取配置 -> 类型: [%s], 端口/IP: [%s]", type.c_str(), port.c_str());

    // 4. 将读到的端口丢给工厂去干活
    robot_ = robot_go_target::createRobot(type, port, this->get_logger());
    if (!robot_) {
        RCLCPP_ERROR(this->get_logger(), "机器人实例化失败，请检查连接或参数！");
    }

    last_cmd_time_ = this->now();

    // =========================================================
    // 注册各类 ROS 2 话题和定时器
    // =========================================================
    robot_move_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "cmd_vel", 10,
        std::bind(&UniversalRobotDriver::cmdVelCallback, this, std::placeholders::_1));
        
    scan_config_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "scan_config", 10,
        std::bind(&UniversalRobotDriver::scanconfigCallback, this, std::placeholders::_1));
    // 创建扫查控制指令的订阅者 (订阅 "scan_cmd" 话题)    
    scan_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "scan_cmd", 10, std::bind(&UniversalRobotDriver::scan_cmd_callback, this, std::placeholders::_1));
        
    robot_distance_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "robot_distance_move", 10,
        std::bind(&UniversalRobotDriver::stepMoveCallback, this, std::placeholders::_1));
        
    robot_rotate_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "robot_rotate_move", 10,
        std::bind(&UniversalRobotDriver::stepRotateCallback, this, std::placeholders::_1));
        
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("agv_pose", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&UniversalRobotDriver::timerCallback, this)
    );
    // 创建一个 20 毫秒 (50Hz) 的高频定时器，用来推动底层状态机
    state_machine_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), 
        std::bind(&UniversalRobotDriver::state_machine_callback, this));
}

UniversalRobotDriver::~UniversalRobotDriver() 
{
    std::lock_guard<std::mutex> lock(robot_mutex_);
    if(robot_) 
    {
        RCLCPP_INFO(this->get_logger(), "节点关闭，停止机器人...");
        robot_->setVelocity(0.0, 0.0, 0.0);
        robot_->moveshutdown();
    }
}

void UniversalRobotDriver::cmdVelCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{
    std::lock_guard<std::mutex> lock(this->robot_mutex_);
    if (robot_ && msg->data.size() >= 3) 
    {
        robot_->setVelocity(msg->data[0], msg->data[1], msg->data[2]);
                
        RCLCPP_INFO(get_logger(), "cmd_vel订阅成功 vx=%.3f, vy=%.3f, wz=%.3f", msg->data[0], msg->data[1], msg->data[2]);
        last_cmd_time_ = this->now();
    }
}

void UniversalRobotDriver::scanconfigCallback(const geometry_msgs::msg::Twist::SharedPtr msg) 
{
    std::shared_ptr<RobotMove> safe_robot;
    {
        std::lock_guard<std::mutex> lock(this->robot_mutex_);
        safe_robot = this->robot_;
    }
    if (!safe_robot) return;

    auto scanner_robot = std::dynamic_pointer_cast<IScannerController>(safe_robot);
    if (scanner_robot) 
    {
        double speed = msg->linear.x;
        double distance = msg->linear.y;
        double precision = msg->linear.z;
        
        uint16_t param_ch7 = static_cast<uint16_t>(msg->angular.x); 
        uint16_t param_ch8 = static_cast<uint16_t>(msg->angular.y);

        int scan_times = static_cast<int>(msg->angular.z);

        RCLCPP_INFO(this->get_logger(), "配置扫查器: 速度=%.2f, 距离=%.2f, 精度=%.2f, M1=%d, M2=%d, 次数=%d", 
                    speed, distance, precision, param_ch7, param_ch8, scan_times);
        scanner_robot->setScannerConfig(speed, distance, precision, param_ch7, param_ch8,scan_times);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "当前机型不支持扫查器配置，指令已忽略。");
    }
}

// ==========================================
// 扫查启动/停止的话题回调
// ==========================================
void UniversalRobotDriver::scan_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(this->robot_mutex_);

    // 尝试将当前机器人对象强转为扫查器接口
    auto scanner_ptr = std::dynamic_pointer_cast<IScannerController>(robot_);
    if (scanner_ptr) {
        // 如果当前机型（如涵道）支持扫查功能，调用 triggerScan
        scanner_ptr->triggerScan(msg->data);
    } else {
        RCLCPP_WARN(this->get_logger(), "当前连接的机型不支持扫查功能！");
    }
}

void UniversalRobotDriver::stepMoveCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::shared_ptr<RobotMove> safe_robot;
    {
        std::lock_guard<std::mutex> lock(this->robot_mutex_);
        safe_robot = this->robot_;
    }
    if (!safe_robot) return;

    auto nav_robot = std::dynamic_pointer_cast<IAdvancedNavigation>(safe_robot);
    if (nav_robot)
    {
        double dist = msg->linear.z;
        double vx = msg->linear.x;
        double vy = msg->linear.y;
        RCLCPP_INFO(this->get_logger(), "StepMove: dist=%.2f...", dist);
        
        std::thread([nav_robot, dist, vx, vy]() {
            nav_robot->movebydistance(dist, vx, vy);
        }).detach();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "当前机型不支持高级导航定距移动！");
    }
}

void UniversalRobotDriver::stepRotateCallback(const geometry_msgs::msg::Twist::SharedPtr msg) 
{
    std::shared_ptr<RobotMove> safe_robot;
    {
        std::lock_guard<std::mutex> lock(this->robot_mutex_);
        safe_robot = this->robot_;
    }
    if (!safe_robot) return;

    auto nav_robot = std::dynamic_pointer_cast<IAdvancedNavigation>(safe_robot);
    if (nav_robot)
    {
        double angle = msg->angular.z;
        double vw = msg->angular.y; 

        RCLCPP_INFO(this->get_logger(), "StepRotate: angle=%.2f, vw=%.2f", angle, vw);

        std::thread([nav_robot, angle, vw]()
        {
            nav_robot->rotatebyangle(angle, vw);
        }).detach();
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "当前机型不支持高级导航定角转动，指令已忽略。");
    }
}

void UniversalRobotDriver::timerCallback()
{
    std::lock_guard<std::mutex> lock(this->robot_mutex_);
    if (robot_) 
    {
        robot_->send_loop(); 
        
        auto pose_provider = std::dynamic_pointer_cast<IPoseProvider>(robot_);
        if (pose_provider) 
        {
            static int loop_count = 0;
            if (++loop_count >= 5) 
            {
                loop_count = 0;
                double x = 0, y = 0, angle = 0;
                if (pose_provider->getpose(x, y, angle)) 
                {
                    auto msg = geometry_msgs::msg::Pose2D();
                    msg.x = x; msg.y = y; msg.theta = angle;
                    pose_pub_->publish(msg);
                    RCLCPP_DEBUG(this->get_logger(), "Published pose: x=%.3f, y=%.3f", x, y);
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to get pose from provider.");
                }
            }
        }
        else
        {
            RCLCPP_ERROR_ONCE(this->get_logger(), "Robot object does not implement IPoseProvider!");
        }
    }
}

// ==========================================
// 定时器回调：整个系统的心跳，驱动 scannercontrol 运转
// ==========================================
void UniversalRobotDriver::state_machine_callback() {
    std::lock_guard<std::mutex> lock(this->robot_mutex_);
    if (!robot_) return;

    // 1. 推动扫查状态机运行
    auto scanner_ptr = std::dynamic_pointer_cast<IScannerController>(robot_);
    if (scanner_ptr) {
        // 这里就是关键！每隔 20ms 调用一次，状态机里的 switch-case 就会被执行一次
        scanner_ptr->scannercontrol(); 
    }

    // 2. (可选) 如果你还有其他的定期任务，比如向外发布里程计 TF 树、心跳包等，也可以写在这里
}