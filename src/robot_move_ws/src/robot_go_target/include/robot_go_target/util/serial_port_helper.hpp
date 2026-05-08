#ifndef SERIAL_PORT_HELPER_HPP
#define SERIAL_PORT_HELPER_HPP

#include <string>

namespace robot_go_target {
namespace util {

class SerialPortHelper {
public:
    /**
     * @brief 打开并配置 Linux 串口 (使用 termios2 支持非标准波特率)
     * @param port 串口路径，例如 "/dev/ttyUSB0"
     * @param baudrate 波特率，例如 100000 或 115200
     * @param use_8E2 是否使用 8E2(8数据位, 偶校验, 2停止位)。若为 false，则为常见的 8N1。
     * @return 成功返回文件描述符(fd)，失败返回 -1
     */
    static int openAndConfigure(const std::string& port, int baudrate, bool use_8E2);
};

} // namespace util
} // namespace robot_go_target

#endif // SERIAL_PORT_HELPER_HPP