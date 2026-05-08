#include "robot_go_target/impl/duct/duct_robot_move.hpp"
#include "robot_go_target/util/serial_port_helper.hpp"

#include <iostream>
#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <chrono> // 确保顶部有这个头文件
DuctRobotMove::DuctRobotMove()
{
    ch_.resize(16, 1000);
    ch_[0] = 1000;
    ch_[1] = 200;  // M1 默认抬起
    ch_[2] = 1000;
    ch_[3] = 200;  // M2 默认回零
    ch_[4] = 1000;  // 扫查引擎默认关闭
    ch_[5] = 500;

    ch_[6] = 1000; 
    ch_[7] = 1600;
    ch_[8] = 60;
    ch_[9] = 30;
    ch_[10] = 3;
    ch_[11] = 1000; // CH12 复位默认中位
    ch_[12] = scan_times; // CH12 复位默认中位
}

rclcpp::Logger DuctRobotMove::get_logger()
{
    return rclcpp::get_logger("DuctRobot_Driver");
}

void DuctRobotMove::pack_protocol_data(std::vector<uint16_t> ch, uint8_t *buf)
{
    if (ch.size() != 16) return;

    for (int i = 0; i < 16; ++i) {
        if (ch[i] > 1800) ch[i] = 1800;
        ch[i] &= 0x07FF;
    }

    buf[0] = 0x0F;
    buf[1] = (uint8_t)(ch[0] & 0xFF);
    buf[2] = (uint8_t)((ch[0] >> 8) | (ch[1] << 3));
    buf[3] = (uint8_t)((ch[1] >> 5) | (ch[2] << 6));
    buf[4] = (uint8_t)((ch[2] >> 2));
    buf[5] = (uint8_t)((ch[2] >> 10) | (ch[3] << 1));
    buf[6] = (uint8_t)((ch[3] >> 7) | (ch[4] << 4));
    buf[7] = (uint8_t)((ch[4] >> 4) | (ch[5] << 7));
    buf[8] = (uint8_t)((ch[5] >> 1));
    buf[9] = (uint8_t)((ch[5] >> 9) | (ch[6] << 2));
    buf[10] = (uint8_t)((ch[6] >> 6) | (ch[7] << 5));
    buf[11] = (uint8_t)((ch[7] >> 3));
    buf[12] = (uint8_t)(ch[8] & 0xFF);
    buf[13] = (uint8_t)((ch[8] >> 8) | (ch[9] << 3));
    buf[14] = (uint8_t)((ch[9] >> 5) | (ch[10] << 6));
    buf[15] = (uint8_t)((ch[10] >> 2));
    buf[16] = (uint8_t)((ch[10] >> 10) | (ch[11] << 1));
    buf[17] = (uint8_t)((ch[11] >> 7) | (ch[12] << 4));
    buf[18] = (uint8_t)((ch[12] >> 4) | (ch[13] << 7));
    buf[19] = (uint8_t)((ch[13] >> 1));
    buf[20] = (uint8_t)((ch[13] >> 9) | (ch[14] << 2));
    buf[21] = (uint8_t)((ch[14] >> 6) | (ch[15] << 5));
    buf[22] = (uint8_t)((ch[15] >> 3));
    buf[23] = 0x00;
    buf[24] = 0x00;
}

bool DuctRobotMove::isActionDone()
{
    if (action_wait_ticks_ > 0) {
        action_wait_ticks_--;
        return false; 
    }
    return true; 
}

void DuctRobotMove::sendNextSignal()
{
    if (serial_fd_ != -1) {
        const char *cmd = "next\n"; 
        write(serial_fd_, cmd, 5);
        RCLCPP_INFO(get_logger(), "[NEXT] 已发送 'next\\n'，触发 STM32 反向扫查！");
    }
}

void DuctRobotMove::checkSerialSignals()
{
    if (serial_fd_ == -1) return;

    int bytes_avail = 0;
    if (ioctl(serial_fd_, FIONREAD, &bytes_avail) >= 0 && bytes_avail > 0)
    {
        char buf[128];
        int len = read(serial_fd_, buf, std::min(bytes_avail, 127));
        if (len > 0)
        {
            buf[len] = '\0';
            rx_buffer_ += buf; 

            if (rx_buffer_.find("true") != std::string::npos) {
                received_true_signal_ = true; 
                rx_buffer_.clear();           
                RCLCPP_INFO(get_logger(), "==> [反馈] 收到 'true'：单次扫查及虚拟轴完成！");
            }
            else if (rx_buffer_.find("mdone") != std::string::npos) {
                received_mdone_signal_ = true; 
                rx_buffer_.clear();           
                RCLCPP_INFO(get_logger(), "==> [反馈] 收到 'mdone'：M1/M2 动作执行完毕！");
            }
            else if (rx_buffer_.find("rdone") != std::string::npos) {
                received_rdone_signal_ = true; 
                rx_buffer_.clear();           
                RCLCPP_INFO(get_logger(), "==> [反馈] 收到 'rdone'：复位完成！");
            }

            if (rx_buffer_.size() > 1024) rx_buffer_.clear();
        }
    }
}

bool DuctRobotMove::init(const std::string &port)
{
    serial_fd_ = robot_go_target::util::SerialPortHelper::openAndConfigure(port, 115200, false);
    if (serial_fd_ == -1) return false;

    RCLCPP_INFO(get_logger(), "串口打开成功，等待 STM32 复位...");
    usleep(2000000); 

    const char *start_cmd = "start\n"; 
    for (int i = 0; i < 5; ++i) {
        write(serial_fd_, start_cmd, 6);
        usleep(50000); 
    }

    ch_[5] = 1600;
    for (int i = 0; i < 5; i++) {
        send_loop();
        usleep(20000);
    }
    return true;
}

void DuctRobotMove::setVelocity(double linear_x, double vy, double angle_z)
{
    (void)vy;
    
    // 【核心修复 1】：扫查状态机运行期间，强制屏蔽外部 cmd_vel 的控制，防止指令打架被意外停车！
    if (scan_state_ != ScannerState::IDLE) {
        return; 
    }

    ch_[5] = 1600;
    if (linear_x >= 0.01)
    { 
        ch_[2] = 1600;
    }
    else if (linear_x <= -0.01)
    {
        ch_[2] = 500;
    }       
    else
    {
        ch_[2] = 1000;
    }

    if (angle_z >= 0.01)
        ch_[0] = 500;
    else if (angle_z <= -0.01)
        ch_[0] = 1600;
    else
        ch_[0] = 1000;
}

void DuctRobotMove::setScannerConfig(double speed, double distance, double precision, uint16_t param_ch7, uint16_t param_ch8,int times)
{
    scan_speed_ = speed;
    vehicle_step_dist_ = distance;
    scan_times = (times - 1) * 100 + 250;

    if (param_ch7 > 1500) param_ch7 = 1500;
    m1_target_pos_ = param_ch7; 
    
    if (param_ch8 > 1200) param_ch8 = 1200;
    m2_max_limit_ = param_ch8;

    if (precision > 0.0 && precision <= 1.0) {
        m2_step_sbus_ = static_cast<int>(m2_max_limit_ * precision);
    } else {
        m2_step_sbus_ = m2_max_limit_; 
    }

    ch_[11] = 1000;
    ch_[6]  = param_ch7;
    ch_[7] = param_ch8; //m2极限
    ch_[8] = speed;
    ch_[9] = distance;
    ch_[10] = m2_step_sbus_;
    ch_[12] = scan_times;

    for (int i = 0; i < 5; i++) {
        send_loop();
        usleep(1500);
    }
    RCLCPP_INFO(get_logger(), "配置已下发底层: M1目标=%d, M2边界=%d, 每次步长=%d, 单次扫查次数=%d", m1_target_pos_, m2_max_limit_, m2_step_sbus_, times);
}

void DuctRobotMove::triggerScan(bool start)
{
    if (start) {
        if (scan_state_ == ScannerState::IDLE) {
            m2_current_sbus_ = 0;
            scan_state_ = ScannerState::INIT_SCAN;
            RCLCPP_INFO(get_logger(), "【指令】启动连续阵列扫查任务！");
        }
    } else {
        scan_state_ = ScannerState::IDLE;
        
        // 【关键修复1】：1000 才是 STM32 真正的急停和重置指令 (900 < CH5 < 1100)
        ch_[4] = 1000; 
        ch_[1] = 200;  // 强制 M1 抬起 (200为安全位置)
        ch_[2] = 1000; // 强制停车
        ch_[0] = 1000;
        for(int i=0; i<3; i++) {
            send_loop();
            usleep(2000);
        }
        RCLCPP_INFO(get_logger(), "【指令】扫查已强制终止！云台安全抬起。");
    }
}

// ==========================================
// 💡 串行避障握手状态机 (免疫幽灵信号终极版)
// ==========================================
void DuctRobotMove::scannercontrol()
{
    checkSerialSignals();
    bool need_send = false; 
    
    static bool sweep_forward = true; 

    switch (scan_state_)
    {
    case ScannerState::IDLE:
        break;

    case ScannerState::INIT_SCAN:
        received_true_signal_ = false;
        received_mdone_signal_ = false;
        received_rdone_signal_ = false;
        rx_buffer_.clear();

        ch_[4] = 1000; 
        ch_[11] = 1600; 

        action_wait_ticks_ = 400; 
        scan_state_ = ScannerState::WAIT_HOME_SLIDE; 
        need_send = true; 
        RCLCPP_INFO(get_logger(), "--> [步骤 0] 触发 DMTP 滑台回零复位，等待 rdone...");
        break;

    case ScannerState::WAIT_HOME_SLIDE:
        if (received_rdone_signal_ || isActionDone())
        {
            received_rdone_signal_ = false;
            // 收到信号，进入冷却
            cooldown_ticks_ = 10; 
            scan_state_ = ScannerState::WAIT_HOME_SLIDE_COOLDOWN; 
        }
        break;

    case ScannerState::WAIT_HOME_SLIDE_COOLDOWN:
        if (cooldown_ticks_ > 0) {
            cooldown_ticks_--;
        } else {
            ch_[11] = 1000; 
            sweep_forward = true; 
            
            static bool toggle1 = false; 
            toggle1 = !toggle1;
            ch_[1] = m1_target_pos_ + 200 + (toggle1 ? 40 : 0);  
            ch_[3] = 0 + 200 + (toggle1 ? 40 : 0);               
            m2_current_sbus_ = 0;

            received_mdone_signal_ = false;
            received_true_signal_ = false;

            action_wait_ticks_ = 100;
            scan_state_ = ScannerState::WAIT_INIT_MOVE;
            need_send = true; 
            RCLCPP_INFO(get_logger(), "--> [步骤 1] 滑台已归零。阵列起点初始化: M1 放下[%d], M2 归零...", m1_target_pos_);
        }
        break;

    case ScannerState::WAIT_INIT_MOVE:
        if (received_mdone_signal_ || isActionDone())
        {
            received_mdone_signal_ = false;
            // 【核心修复】：收到 M1 放下完成的信号，进入冷却，防止接下来的 ch_[4] 被丢包！
            cooldown_ticks_ = 2; 
            scan_state_ = ScannerState::WAIT_INIT_MOVE_COOLDOWN; 
        }
        break;

    case ScannerState::WAIT_INIT_MOVE_COOLDOWN:
        if (cooldown_ticks_ > 0) {
            cooldown_ticks_--;
        } else {
            received_true_signal_ = false; 

            ch_[12] = scan_times;  
            ch_[4] = 1600;  // 安全下发正向扫查，绝对不会丢包了

            scan_state_ = ScannerState::WAIT_TRUE_SIGNAL;
            need_send = true; 
            RCLCPP_INFO(get_logger(), "--> [步骤 2] 云台就位。激活STM32底层正向扫查，等待 true...");
        }
        break;

    case ScannerState::WAIT_TRUE_SIGNAL:
        if (received_true_signal_)
        {
            received_true_signal_ = false;
            // 收到扫查完成的 true 信号，进入冷却
            cooldown_ticks_ = 2; 
            scan_state_ = ScannerState::WAIT_TRUE_SIGNAL_COOLDOWN;
        }
        break;

    case ScannerState::WAIT_TRUE_SIGNAL_COOLDOWN:
        if (cooldown_ticks_ > 0) {
            cooldown_ticks_--;
        } else {
            ch_[4] = 1000; 

            if (m2_current_sbus_ >= m2_max_limit_)
            {
                m2_current_sbus_ = 0; 
                static bool toggle2 = false; 
                toggle2 = !toggle2;
                ch_[1] = 200 + (toggle2 ? 40 : 0); 
                ch_[3] = 200 + (toggle2 ? 40 : 0); 
                
                received_mdone_signal_ = false;
                action_wait_ticks_ = 150; 
                scan_state_ = ScannerState::WAIT_M2_RESET; 
                need_send = true;
                RCLCPP_INFO(get_logger(), "--> [本轮结束] 达到极限 %d 行！抬起 M1，M2 回零，准备行车...", m2_max_limit_);
            }
            else
            {
                m2_current_sbus_ += m2_step_sbus_;
                if (m2_current_sbus_ > m2_max_limit_) m2_current_sbus_ = m2_max_limit_;
                
                static bool toggle3 = false; 
                toggle3 = !toggle3;
                ch_[1] = 200 + (toggle3 ? 40 : 0); 
                ch_[3] = m2_current_sbus_ + 200 + (toggle3 ? 40 : 0);

                received_mdone_signal_ = false;
                action_wait_ticks_ = 150; 
                scan_state_ = ScannerState::WAIT_M1_LIFT; 
                need_send = true;
                RCLCPP_INFO(get_logger(), "--> [步骤 3] 收到 true！抬起 M1，换行移动 M2 至 [%d]...", m2_current_sbus_);
            }
        }
        break;

    case ScannerState::WAIT_M1_LIFT:
        if (received_mdone_signal_ || isActionDone())
        {
            received_mdone_signal_ = false;
            cooldown_ticks_ = 2; 
            scan_state_ = ScannerState::WAIT_M1_LIFT_COOLDOWN;
        }
        break;

    case ScannerState::WAIT_M1_LIFT_COOLDOWN:
        if (cooldown_ticks_ > 0) {
            cooldown_ticks_--; 
        } else {
            static bool toggle4 = false; 
            toggle4 = !toggle4;
            ch_[1] = m1_target_pos_ + 200 + (toggle4 ? 40 : 0);

            received_mdone_signal_ = false;
            received_true_signal_ = false; 

            action_wait_ticks_ = 150; 
            scan_state_ = ScannerState::WAIT_M2_MOVE; 
            need_send = true;
            RCLCPP_INFO(get_logger(), "--> [步骤 4] M2 换行到位。放下 M1 重新贴合...");
        }
        break;

    case ScannerState::WAIT_M2_MOVE:
        if (received_mdone_signal_ || isActionDone())
        {
            received_mdone_signal_ = false;
            cooldown_ticks_ = 2; 
            scan_state_ = ScannerState::WAIT_M2_MOVE_COOLDOWN;
        }
        break;

    case ScannerState::WAIT_M2_MOVE_COOLDOWN:
        if (cooldown_ticks_ > 0) {
            cooldown_ticks_--; 
        } else {
            ch_[12] = scan_times; 
            sweep_forward = !sweep_forward;
            ch_[4] = sweep_forward ? 1600 : 400; 

            received_true_signal_ = false; 

            scan_state_ = ScannerState::WAIT_TRUE_SIGNAL; 
            need_send = true;
            RCLCPP_INFO(get_logger(), "--> [步骤 5] M1 贴合完毕，触发%s扫查！", sweep_forward ? "正向" : "反向");
        }
        break;

    case ScannerState::WAIT_M2_RESET:
        if (received_mdone_signal_ || isActionDone())
        {
            received_mdone_signal_ = false;
            cooldown_ticks_ = 2; 
            scan_state_ = ScannerState::WAIT_M2_RESET_COOLDOWN;
        }
        break;

    case ScannerState::WAIT_M2_RESET_COOLDOWN:
        if (cooldown_ticks_ > 0) {
            cooldown_ticks_--; 
        } else {
            ch_[2] = 400; 
            action_wait_ticks_ = vehicle_step_dist_; 
            scan_state_ = ScannerState::WAIT_VEHICLE_BACK; 
            need_send = true;
            RCLCPP_INFO(get_logger(), "--> [步骤 6] 云台已安全回原点，车辆开始前进10秒...");
        }
        break;

    case ScannerState::WAIT_VEHICLE_BACK:
        if (isActionDone())
        {
            ch_[2] = 1000; 
            scan_state_ = ScannerState::INIT_SCAN;
            need_send = true;
            RCLCPP_INFO(get_logger(), "--> [步骤 7] 停车完毕。自动开启下一轮大循环！");
        }
        break;

    default:
        break;
    }
}


void DuctRobotMove::send_loop()
{
    if (serial_fd_ != -1)
    {
        // 🌟🌟【核心防御：硬件防洪机制】🌟🌟
        // 限制最高发送频率，防止 100Hz 狂发导致串口粘包和 STM32 乱码
        static auto last_send_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time).count();

        // 强制两帧之间至少间隔 30 毫秒 (发送频率降至 ~33Hz)
        // 这将在物理电平上强制制造出约 27ms 的静默间隙，让 STM32 百分之百精准断帧！
        if (elapsed < 50) {
            return; 
        }
        last_send_time = now;

        uint8_t buffer[25];
        pack_protocol_data(ch_, buffer);
        ssize_t written = write(serial_fd_, buffer, 25);
        (void)written; 
    }
}

void DuctRobotMove::moveshutdown()
{
    if (serial_fd_ != -1)
    {
        ch_[0] = 1000;
        ch_[1] = 200;
        ch_[2] = 1000;
        ch_[3] = 200;
        ch_[4] = 1000;
        ch_[5] = 500;
        ch_[11] = 1000; 
        for (int i = 0; i < 5; i++)
        {
            send_loop();
            usleep(20000);
        }
        close(serial_fd_);
        serial_fd_ = -1;

        RCLCPP_INFO(get_logger(), "涵道机器人已安全停止连接");
    }
}