#include "robot_go_target/impl/air/air_robot_move.hpp"
#include "robot_go_target/util/serial_port_helper.hpp"

#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <unistd.h>

AirRobotMove::AirRobotMove()
{
    ch_.resize(16);
    for(int i = 0; i < 16; i++) ch_[i] = STOP.at(i);
}

rclcpp::Logger AirRobotMove::get_logger() {
    return rclcpp::get_logger("AirRobot_Driver");
}

void AirRobotMove::pack_protocol_data(std::vector<uint16_t> ch, uint8_t* buf) 
{
    if (ch.size() != 16) 
    {
        RCLCPP_ERROR(get_logger(), "需要 16 个通道数据");
        return ;
    }
    for (int i = 0; i < 16; ++i) 
    {
        if(ch[i] > 2047) ch[i] = 2047;
    }

    buf[0] = 0x0F; // 帧头
    
    // 前 8 通道
    buf[1]  = (uint8_t)((ch[0] >> 3) & 0xFF);
    buf[2]  = (uint8_t)(((ch[0] << 5) | (ch[1] >> 6)) & 0xFF);
    buf[3]  = (uint8_t)(((ch[1] << 2) | (ch[2] >> 9)) & 0xFF);
    buf[4]  = (uint8_t)((ch[2] >> 1) & 0xFF);
    buf[5]  = (uint8_t)(((ch[2] << 7) | (ch[3] >> 4)) & 0xFF);
    buf[6]  = (uint8_t)(((ch[3] << 4) | (ch[4] >> 7)) & 0xFF);
    buf[7]  = (uint8_t)(((ch[4] << 1) | (ch[5] >> 10)) & 0xFF);
    buf[8]  = (uint8_t)((ch[5] >> 2) & 0xFF);
    buf[9]  = (uint8_t)(((ch[5] << 6) | (ch[6] >> 5)) & 0xFF);
    buf[10] = (uint8_t)(((ch[6] << 3) | (ch[7] >> 8)) & 0xFF);
    buf[11] = (uint8_t)(ch[7] & 0xFF);
    
    // 后 8 通道
    buf[12] = (uint8_t)((ch[8] >> 3) & 0xFF);
    buf[13] = (uint8_t)(((ch[8] << 5) | (ch[9] >> 6)) & 0xFF);
    buf[14] = (uint8_t)(((ch[9] << 2) | (ch[10] >> 9)) & 0xFF);
    buf[15] = (uint8_t)((ch[10] >> 1) & 0xFF);
    buf[16] = (uint8_t)(((ch[10] << 7) | (ch[11] >> 4)) & 0xFF);
    buf[17] = (uint8_t)(((ch[11] << 4) | (ch[12] >> 7)) & 0xFF);
    buf[18] = (uint8_t)(((ch[12] << 1) | (ch[13] >> 10)) & 0xFF);
    buf[19] = (uint8_t)((ch[13] >> 2) & 0xFF);
    buf[20] = (uint8_t)(((ch[13] << 6) | (ch[14] >> 5)) & 0xFF);
    buf[21] = (uint8_t)(((ch[14] << 3) | (ch[15] >> 8)) & 0xFF);
    buf[22] = (uint8_t)(ch[15] & 0xFF);
    
    // Flag
    buf[23] = 0x00;
    
    // 和校验 (Sum 0-23) & 0xFF
    unsigned int sum = 0;
    for(int i=0; i<24; i++) sum += buf[i];
    buf[24] = (uint8_t)(sum & 0xFF);
}

bool AirRobotMove::init(const std::string& port)
{
    // 调用工具类，一行搞定！参数: 端口, 波特率 100000, 开启 8E2 模式
    serial_fd_ = robot_go_target::util::SerialPortHelper::openAndConfigure(port, 100000, true);
    
    if(serial_fd_ == -1)
    {
        RCLCPP_ERROR(get_logger(), "串口打开或配置失败: %s", port.c_str());
        return false;
    }
    
    RCLCPP_INFO(get_logger(), "气吸附底盘串口初始化成功 (100k, 8E2): %s", port.c_str());
    return true;
}

void AirRobotMove::setVelocity(double linear_x, double vy, double angle_z)
{
    (void)vy;
    const double LINEAR_SCALE = 23000.0;
    
    if(linear_x > 0.001) 
    {
        int val = 1500 - (int)(linear_x * LINEAR_SCALE);
        if(val <= 1050) val = 1050; 
        ch_[1] = (uint16_t)val;
    }
    else if(linear_x < -0.001) 
    {
        int val = 1500 + (int)(std::abs(linear_x) * LINEAR_SCALE);
        if(val >= 1950) val = 1950;
        ch_[1] = (uint16_t)val;
    }
    else ch_[1] = 1500;

    if(angle_z > 0.001) 
    {
        const double LEFT_SCALE = 30000.0;
        int val = 1500 - (int)(angle_z * LEFT_SCALE);
        if(val <= 1050) val = 1050;
        ch_[0] = (uint16_t)val;
    }
    else if(angle_z < -0.001) 
    {
        const double RIGHT_SCALE = 40000.0;
        int val = 1500 + (int)(std::abs(angle_z) * RIGHT_SCALE);
        if(val >= 1950) val = 1950;
        ch_[0] = (uint16_t)val;
    }
    else ch_[0] = 1500;
}

void AirRobotMove::send_loop()
{
    if(serial_fd_ != -1)
    {
        uint8_t buffer[25];
        pack_protocol_data(ch_, buffer);
        write(serial_fd_, buffer, 25);
    }
}

void AirRobotMove::moveshutdown()
{
    if(serial_fd_ != -1)
    {
        close(serial_fd_);
        serial_fd_ = -1;
        RCLCPP_INFO(get_logger(), "气吸附机器人底盘已关闭");
    }
}