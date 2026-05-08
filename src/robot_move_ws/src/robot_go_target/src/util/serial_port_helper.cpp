#include "robot_go_target/util/serial_port_helper.hpp"

#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace robot_go_target {
namespace util {

int SerialPortHelper::openAndConfigure(const std::string& port, int baudrate, bool use_8E2)
{
    // 打开串口
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;
    
    struct termios2 options;
    if (ioctl(fd, TCGETS2, &options) < 0) {
        close(fd);
        return -1;
    }

    // 基础配置与自定义波特率
    options.c_cflag &= ~CBAUD;   
    options.c_cflag |= BOTHER;   
    options.c_ispeed = baudrate;   
    options.c_ospeed = baudrate;   
    
    // 8位数据位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    if (use_8E2) 
    {
        // 配置为 8E2 (偶校验，2个停止位) —— 用于气吸附和涵道 SBUS
        options.c_cflag |= PARENB;  
        options.c_cflag &= ~PARODD; 
        options.c_iflag |= (INPCK | ISTRIP); 
        options.c_cflag |= CSTOPB;
    } 
    else 
    {
        // 配置为 8N1 (无校验，1个停止位) —— 用于磁吸附
        options.c_cflag &= ~PARENB;  
        options.c_iflag &= ~(INPCK | ISTRIP); 
        options.c_cflag &= ~CSTOPB;
    }

    // 其他通用配置 (RAW 模式)
    options.c_cflag |= (CLOCAL | CREAD); 
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    options.c_oflag &= ~OPOST; 
    options.c_iflag &= ~(IXON | IXOFF | IXANY); 

    // 应用配置
    if (ioctl(fd, TCSETS2, &options) < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

} // namespace util
} // namespace robot_go_target