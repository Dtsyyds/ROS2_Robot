#include "robot_go_target/impl/mag/mag_robot_move.hpp"
#include "robot_go_target/util/serial_port_helper.hpp"

#include <iostream>
#include <algorithm>
#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sstream>

MagRobotMove::MagRobotMove()
{
    ch_.resize(16, VAL_MID);
}

rclcpp::Logger MagRobotMove::get_logger() {
    return rclcpp::get_logger("MagRobot_Driver");
}

void MagRobotMove::pack_protocol_data(const std::vector<uint16_t>& ch, uint8_t* buf) 
{
    if (ch.size() != 16) return;
    std::vector<uint16_t> clean_ch = ch;
    for(auto& val : clean_ch) { if(val > 2047) val = 2047; }

    buf[0] = 0x0F; 
    buf[1]  = (uint8_t)((clean_ch[0] & 0x07FF));
    buf[2]  = (uint8_t)((clean_ch[0] & 0x07FF) >> 8 | (clean_ch[1] & 0x07FF) << 3);
    buf[3]  = (uint8_t)((clean_ch[1] & 0x07FF) >> 5 | (clean_ch[2] & 0x07FF) << 6);
    buf[4]  = (uint8_t)((clean_ch[2] & 0x07FF) >> 2);
    buf[5]  = (uint8_t)((clean_ch[2] & 0x07FF) >> 10 | (clean_ch[3] & 0x07FF) << 1);
    buf[6]  = (uint8_t)((clean_ch[3] & 0x07FF) >> 7 | (clean_ch[4] & 0x07FF) << 4);
    buf[7]  = (uint8_t)((clean_ch[4] & 0x07FF) >> 4 | (clean_ch[5] & 0x07FF) << 7);
    buf[8]  = (uint8_t)((clean_ch[5] & 0x07FF) >> 1);
    buf[9]  = (uint8_t)((clean_ch[5] & 0x07FF) >> 9 | (clean_ch[6] & 0x07FF) << 2);
    buf[10] = (uint8_t)((clean_ch[6] & 0x07FF) >> 6 | (clean_ch[7] & 0x07FF) << 5);
    buf[11] = (uint8_t)((clean_ch[7] & 0x07FF) >> 3);
    buf[12] = (uint8_t)((clean_ch[8] & 0x07FF));
    buf[13] = (uint8_t)((clean_ch[8] & 0x07FF) >> 8 | (clean_ch[9] & 0x07FF) << 3);
    buf[14] = (uint8_t)((clean_ch[9] & 0x07FF) >> 5 | (clean_ch[10] & 0x07FF) << 6);
    buf[15] = (uint8_t)((clean_ch[10] & 0x07FF) >> 2);
    buf[16] = (uint8_t)((clean_ch[10] & 0x07FF) >> 10 | (clean_ch[11] & 0x07FF) << 1);
    buf[17] = (uint8_t)((clean_ch[11] & 0x07FF) >> 7 | (clean_ch[12] & 0x07FF) << 4);
    buf[18] = (uint8_t)((clean_ch[12] & 0x07FF) >> 4 | (clean_ch[13] & 0x07FF) << 7);
    buf[19] = (uint8_t)((clean_ch[13] & 0x07FF) >> 1);
    buf[20] = (uint8_t)((clean_ch[13] & 0x07FF) >> 9 | (clean_ch[14] & 0x07FF) << 2);
    buf[21] = (uint8_t)((clean_ch[14] & 0x07FF) >> 6 | (clean_ch[15] & 0x07FF) << 5);
    buf[22] = (uint8_t)((clean_ch[15] & 0x07FF) >> 3);
    buf[23] = 0x00; 
    buf[24] = 0x00; 
}

bool MagRobotMove::init(const std::string& port)
{
    // 调用工具类，参数: 端口, 波特率 100000, 开启 8E2 模式
    serial_fd_ = robot_go_target::util::SerialPortHelper::openAndConfigure(port, 100000, true);
    
    if(serial_fd_ == -1)
    {
        RCLCPP_ERROR(get_logger(), "串口打开或配置失败: %s", port.c_str());
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "磁吸附底盘串口初始化成功 (100k, 8E2): %s", port.c_str());

    ch_[6] = VAL_HIGH;
    ch_[7] = VAL_LOW; 
    for(int i=0; i<20; i++) {
        send_loop();
        usleep(14000);
    }
    
    RCLCPP_INFO(get_logger(), "磁吸附底盘初始化完成 (电机解锁, 低速模式)");
    return true;
}

void MagRobotMove::setVelocity(double linear_x, double vy, double angle_z)
{
    (void)vy; 
    if (linear_x >= 0.01) ch_[1] = 1642;
    else if (linear_x <= -0.01) ch_[1] = 362;
    else ch_[1] = 1002; 

    if (angle_z >= 0.01) ch_[0] = 362;
    else if (angle_z <= -0.01) ch_[0] = 1642;
    else ch_[0] = 1002;
}

void MagRobotMove::send_loop()
{
    if(serial_fd_ != -1)
    {
        uint8_t buffer[25];
        pack_protocol_data(ch_, buffer);
        ssize_t written = write(serial_fd_, buffer, 25);
        (void)written; // 防止编译器报 unused-result 警告
        
        static int debug_cnt = 0;
        if(debug_cnt++ % 50 == 0) 
        {
            std::stringstream hex_stream;
            for(int i = 0; i < 25; i++) 
            {
                char hex_buf[4];
                snprintf(hex_buf, sizeof(hex_buf), "%02X ", buffer[i]);
                hex_stream << hex_buf;
            }
            RCLCPP_INFO(get_logger(), "[SBUS Hex]: %s", hex_stream.str().c_str());
        }
    }
}

void MagRobotMove::moveshutdown()
{
    if(serial_fd_ != -1)
    {
        std::fill(ch_.begin(), ch_.end(), VAL_MID);
        ch_[6] = VAL_LOW; 
        for(int i=0; i<5; i++) { send_loop(); usleep(20000); }
        close(serial_fd_);
        serial_fd_ = -1;
        
        RCLCPP_INFO(get_logger(), "磁吸附机器人已停止并上锁，串口关闭");
    }
}