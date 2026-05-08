#include "robot_go_target/impl/agv/agv_robot_move.hpp"
#include <nlohmann/json.hpp>

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <thread>
#include <chrono>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

AGVRobotMove::AGVRobotMove() {}

rclcpp::Logger AGVRobotMove::get_logger() {
    return rclcpp::get_logger("AGV_Driver");
}

bool AGVRobotMove::connectPort(const std::string& ip, int port, int& sock_fd)
{
    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) return false;

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr)); 
    addr.sin_family = AF_INET; 
    addr.sin_port = htons(port);    
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) return false;

    struct timeval timeout;      
    timeout.tv_sec = 1; timeout.tv_usec = 0;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,sizeof(timeout));
    setsockopt(sock_fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,sizeof(timeout));

    if (connect(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) 
    {
        close(sock_fd);
        sock_fd = -1;
        return false;
    }
    return true;
}

std::vector<uint8_t> AGVRobotMove::packSeerFrame(uint16_t type, const std::string& json_str) 
{
    uint32_t len = json_str.length();
    std::vector<uint8_t> frame(16 + len);
    frame[0] = 0x5A; 
    frame[1] = 0x01; 
    seq_num_++; 
    frame[2] = (seq_num_ >> 8) & 0xFF; 
    frame[3] = seq_num_ & 0xFF;
    frame[4] = (len >> 24) & 0xFF;     
    frame[5] = (len >> 16) & 0xFF;
    frame[6] = (len >> 8) & 0xFF;
    frame[7] = len & 0xFF;
    frame[8] = (type >> 8) & 0xFF;     
    frame[9] = type & 0xFF;
    memset(&frame[10], 0, 6);          
    memcpy(&frame[16], json_str.c_str(), len); 
    return frame;
}

void AGVRobotMove::unlockBrake() 
{
    nlohmann::json j;
    j["id"] = 0;
    j["status"] = true;
    
    std::vector<uint8_t> frame = packSeerFrame(6001, j.dump());
    if (sock_19210_ >= 0) send(sock_19210_, frame.data(), frame.size(), 0);
}

void AGVRobotMove::readresponse(int sock_fd, const std::string& cmd_name) 
{
    if (sock_fd < 0) return;
    char buffer[4096]; 
    memset(buffer, 0, sizeof(buffer));
    int len = recv(sock_fd, buffer, sizeof(buffer)-1, 0);
    if (len > 16) 
    {
        RCLCPP_INFO(get_logger(), "%s Reply: %s", cmd_name.c_str(), buffer + 16);
    }
}

bool AGVRobotMove::getcurrentpose(double& x, double& y, double& theta)
{
    if (sock_19204_ < 0) return false;
    
    // 发送空 JSON 请求
    std::vector<uint8_t> frame = packSeerFrame(1004, "{}");
    send(sock_19204_, frame.data(), frame.size(), 0);

    char buffer[4096]; 
    memset(buffer, 0, sizeof(buffer));
    int len = recv(sock_19204_, buffer, sizeof(buffer)-1, 0);
    
    if(len > 16)
    {
        char *resp_data = buffer + 16;
        try 
        {
            // 现代 C++ JSON 解析
            nlohmann::json json_data = nlohmann::json::parse(resp_data);
            
            // 安全检查字段是否存在且类型正确
            if(json_data.contains("x") && json_data["x"].is_number() &&
               json_data.contains("y") && json_data["y"].is_number() &&
               json_data.contains("angle") && json_data["angle"].is_number())
            {
                x = json_data["x"].get<double>();
                y = json_data["y"].get<double>();
                theta = json_data["angle"].get<double>();
                return true;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "JSON x/y/angle data format error or missing: %s", resp_data);
            }
        } 
        catch (const nlohmann::json::exception& e) 
        {
            RCLCPP_ERROR(get_logger(), "JSON parsing exception: %s | Raw: %s", e.what(), resp_data);
        }
    }
    else if (len == 0)
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *rclcpp::Clock::make_shared(), 5000, "Connection to 19204 closed by robot.");
    }
    else if (len < 0)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *rclcpp::Clock::make_shared(), 5000, "Receive timeout or error on 19204.");
    }
    else
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *rclcpp::Clock::make_shared(), 5000, "Received incomplete header (len=%d) on 19204.", len);
    }
    return false;
}

void AGVRobotMove::robot_control_reloc_req(void)
{
    double x = 0.0, y = 0.0, theta = 0.0;
    if(!getcurrentpose(x, y, theta))
    {
        RCLCPP_ERROR(get_logger(), "重定位失败：获取当前位置失败!!!");
        return;
    }
    
    RCLCPP_INFO(get_logger(), "当前位置: x=%.3f, y=%.3f, theta=%.3f", x, y, theta);

    // 构建 JSON 并发送重定位请求
    nlohmann::json j;
    j["x"] = x;
    j["y"] = y;
    j["theta"] = theta;
    j["length"] = 2; 
    
    std::vector<uint8_t> frame = packSeerFrame(2002, j.dump());
    if (sock_19205_ >= 0) send(sock_19205_, frame.data(), frame.size(), 0);
    
    RCLCPP_INFO(get_logger(), "等待重定位(3s)...");
    sleep(3);

    // 发送确认
    std::vector<uint8_t> comfirm_frame = packSeerFrame(2003, "{}");
    send(sock_19205_, comfirm_frame.data(), comfirm_frame.size(), 0);
    readresponse(sock_19205_, "Reloc Confirm (2003)");
    
    RCLCPP_INFO(get_logger(), "重定位成功!");
}

bool AGVRobotMove::robot_navigation_status()
{
    if(sock_19204_ < 0) return false;
    
    RCLCPP_INFO(get_logger(), "等待导航任务完成...");
    
    while(true)
    {
        nlohmann::json navi_root;
        navi_root["simple"] = true;
        
        std::vector<uint8_t> navi_frame = packSeerFrame(1020, navi_root.dump());
        send(sock_19204_, navi_frame.data(), navi_frame.size(), 0);

        char buf[4096];
        memset(buf, 0, sizeof(buf));
        int len = recv(sock_19204_, buf, 4095, 0);
        
        if (len <= 0) 
        {
            RCLCPP_ERROR(get_logger(), "TCP连接异常或断开，停止等待导航！");
            return false;
        }

        if(len > 16)
        {
            try 
            {
                nlohmann::json resp_data = nlohmann::json::parse(buf + 16);
                
                if(resp_data.contains("task_status") && resp_data["task_status"].is_number_integer())
                {
                    int status = resp_data["task_status"].get<int>();
                    if (status == 0 || status == 4) 
                    { 
                        RCLCPP_INFO(get_logger(), "导航完成 (Status: %d)", status);
                        return true; 
                    }
                    if (status == 5) 
                    { 
                        RCLCPP_ERROR(get_logger(), "导航失败 (Status: 5)!");
                        return false; 
                    }
                }
            } 
            catch (const nlohmann::json::exception& e) 
            {
                RCLCPP_ERROR(get_logger(), "导航响应数据解析失败: %s", e.what());
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool AGVRobotMove::init(const std::string& seer_ip)
{
    std::string ip = seer_ip;
    size_t colon = seer_ip.find(':');
    if (colon != std::string::npos) ip = seer_ip.substr(0, colon);
    
    RCLCPP_INFO(get_logger(), "初始化连接到 %s...", ip.c_str());

    bool s10 = connectPort(ip, 19210, sock_19210_); 
    bool s04 = connectPort(ip, 19204, sock_19204_); 
    bool s05 = connectPort(ip, 19205, sock_19205_); 
    bool s06 = connectPort(ip, 19206, sock_19206_); 

    if (s10) RCLCPP_INFO(get_logger(), "连接19210成功...");
    else RCLCPP_ERROR(get_logger(), "连接19210失败...");
    
    if (s04) RCLCPP_INFO(get_logger(), "连接19204成功...");
    else RCLCPP_ERROR(get_logger(), "连接19204失败...");
    
    if (s05) RCLCPP_INFO(get_logger(), "连接19205成功...");
    else 
    {
        RCLCPP_ERROR(get_logger(), "连接19205失败...");
        return false;
    }
    
    if (s06) RCLCPP_INFO(get_logger(), "连接19206成功...");
    else RCLCPP_ERROR(get_logger(), "连接19206失败...");

    unlockBrake();
    
    RCLCPP_INFO(get_logger(), "初始化端口结束！");
    RCLCPP_INFO(get_logger(), "开始自动重定位...");
    
    robot_control_reloc_req();
    // =======================================================
    // 使用 TF2 记录系统启动那一刻的变换矩阵
    // =======================================================
    double start_x, start_y, start_theta;
    if (getcurrentpose(start_x, start_y, start_theta)) 
    {
        has_started_ = true;
        
        // 1. 构建四元数 (围绕 Z 轴旋转 theta 角)
        tf2::Quaternion q;
        q.setRPY(0, 0, start_theta);
        
        // 2. 设置初始平移和旋转矩阵
        start_transform_.setOrigin(tf2::Vector3(start_x, start_y, 0.0));
        start_transform_.setRotation(q);
        
        RCLCPP_INFO(get_logger(), "已记录 TF 启动原点: x=%.3f, y=%.3f, theta=%.3f rad", start_x, start_y, start_theta);
    } 
    else 
    {
        RCLCPP_WARN(get_logger(), "无法获取初始原点坐标！");
    }

    return true;
}

// 利用 TF2 的逆矩阵乘法来计算相对位姿
bool AGVRobotMove::getpose(double& x, double& y, double& theta)
{
    if (!has_started_) return false;

    double cur_x = 0.0, cur_y = 0.0, cur_theta = 0.0;
    
    if (!getcurrentpose(cur_x, cur_y, cur_theta)) return false;

    tf2::Transform cur_transform;
    tf2::Quaternion cur_q;
    cur_q.setRPY(0, 0, cur_theta);
    cur_transform.setOrigin(tf2::Vector3(cur_x, cur_y, 0.0));
    cur_transform.setRotation(cur_q);

    tf2::Transform relative_transform = start_transform_.inverse() * cur_transform;

    x = relative_transform.getOrigin().x();
    y = relative_transform.getOrigin().y();
    
    tf2::Matrix3x3 mat(relative_transform.getRotation());
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    theta = yaw;

    return true;
}

void AGVRobotMove::setVelocity(double vx, double vy, double wz)
{
    if (sock_19205_ < 0) return;
    
    nlohmann::json j;
    j["vx"] = vx;
    j["vy"] = vy;
    j["w"] = wz;
    j["duration"] = 0; 
    
    std::vector<uint8_t> frame = packSeerFrame(2010, j.dump());
    send(sock_19205_, frame.data(), frame.size(), 0);
}

bool AGVRobotMove::movebydistance(double dist, double vx, double vy)
{ 
    if(sock_19206_ < 0) return false; 
    
    RCLCPP_INFO(get_logger(), "直线运动距离: %.3fm", dist);
    
    nlohmann::json j;
    j["dist"] = dist;
    j["vx"] = vx;
    j["vy"] = vy;
    
    std::vector<uint8_t> frame = packSeerFrame(3055, j.dump());
    send(sock_19206_, frame.data(), frame.size(), 0);
    readresponse(sock_19206_, "平动");
    return robot_navigation_status();
}

bool AGVRobotMove::rotatebyangle(double angle, double vw)
{ 
    if(sock_19206_ < 0) return false; 
    
    RCLCPP_INFO(get_logger(), "转动角度: %.3frad", angle);
    
    nlohmann::json j;
    j["angle"] = angle;
    j["vw"] = vw;
    
    std::vector<uint8_t> frame = packSeerFrame(3056, j.dump());
    send(sock_19206_, frame.data(), frame.size(), 0);
    readresponse(sock_19206_, "转动");
    return robot_navigation_status();
}

void AGVRobotMove::send_loop() {}

void AGVRobotMove::moveshutdown()
{
    if (sock_19204_ >= 0) close(sock_19204_);
    if (sock_19205_ >= 0) close(sock_19205_);
    if (sock_19206_ >= 0) close(sock_19206_);
    if (sock_19210_ >= 0) close(sock_19210_);
    
    RCLCPP_INFO(get_logger(), "AGV 已停止运行！！！");
}