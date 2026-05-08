#ifndef DUCT_ROBOT_MOVE_HPP
#define DUCT_ROBOT_MOVE_HPP

#include "robot_go_target/interface/robot_move_base.hpp"
#include <vector>
#include <string>
#include <cstdint>

// 引入 ROS 2 核心库以使用日志系统
#include <rclcpp/rclcpp.hpp>

// 定义扫查器的运行状态
enum class ScannerState {
    IDLE,               // 待机
    INIT_SCAN,          // 开始扫查 
    WAIT_HOME_SLIDE,    
    WAIT_HOME_SLIDE_COOLDOWN,  // 【新增】
    WAIT_INIT_MOVE,     
    WAIT_INIT_MOVE_COOLDOWN,   // 【新增：解决扫查电机不动的核心】
    WAIT_TRUE_SIGNAL,   
    WAIT_TRUE_SIGNAL_COOLDOWN, // 【新增】
    WAIT_M1_LIFT,       
    WAIT_M1_LIFT_COOLDOWN,     // 已有
    WAIT_M2_MOVE,       
    WAIT_M2_MOVE_COOLDOWN,     // 已有
    WAIT_M2_RESET,      
    WAIT_M2_RESET_COOLDOWN,    // 【新增】
    WAIT_VEHICLE_BACK   
};

class DuctRobotMove : public RobotMove, public IScannerController
{
private:
    ScannerState scan_state_ = ScannerState::IDLE;

    int action_wait_ticks_ = 0;
    int cooldown_ticks_ = 0; // 【新增】：用于替代 usleep 的非阻塞倒计时
    
    // ... 下面保持你原有的变量声明不变 ...
    
    // 扫查核心参数
    int m2_current_sbus_ = 0;      // M2 当前的 SBUS 位置 (初始200代表0位置)
    int m2_step_sbus_ = 600;         // M2 每次移动的 SBUS 步长
    
    uint16_t m1_target_pos_ = 450;  // M1 目标位置 (由通道7设置)
    int m2_max_limit_ = 1200;        // M2 最远边界 (由通道8设置，默认1500)

    // 这些参数现在将由 setScannerConfig() 动态设置
    double scan_speed_ = 60;            
    double vehicle_step_dist_ = 30;      
    double scan_precision_ = 0.5;
    int scan_times = 250;

    bool received_true_signal_ = false;  // 扫查完成信号
    bool received_mdone_signal_ = false; // M1/M2 运动完成信号
    bool received_rdone_signal_ = false; // 扫查器复位完成信号

    int serial_fd_ = -1;
    std::vector<uint16_t> ch_;

    std::string rx_buffer_;        
    void checkSerialSignals();

    /*---------------内部私有方法声明----------------*/ 
    // 辅助函数
    bool isActionDone();             // 判断动作是否完成 (含模拟等待)
    void sendNextSignal();           // 发送 next 信号

    rclcpp::Logger get_logger();
    void pack_protocol_data(std::vector<uint16_t> ch, uint8_t* buf);

public:
    DuctRobotMove();
    ~DuctRobotMove() override = default;

    // 重写接口的公开方法声明
    bool init(const std::string& port) override;
    void setVelocity(double linear_x, double vy, double angle_z) override;
    
    // 触发开关
    void triggerScan(bool start) override;

    // 实现 IScannerController 的方法声明
    void setScannerConfig(double speed, double distance, double precision, uint16_t param_ch7, uint16_t param_ch8, int times) override;
    void scannercontrol() override;
    
    void send_loop() override;
    void moveshutdown() override;
};


#endif // DUCT_ROBOT_MOVE_HPP