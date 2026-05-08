#ifndef ROBOT_MOVE_BASE_HPP
#define ROBOT_MOVE_BASE_HPP

#include <string>
#include <cstdint>
// ==========================================
// 核心能力接口定义 (Interface Segregation)
// ==========================================

// 扫查器控制接口
class IScannerController {
public:
    virtual void setScannerConfig(double speed, double distance, double precision, uint16_t param_ch7, uint16_t param_ch8, int times) = 0;
    // 触发接口，这样外部主控节点就能直接通过基类指针启停扫查器了
    virtual void triggerScan(bool start) = 0;
    virtual void scannercontrol() = 0;
    virtual ~IScannerController() = default;
};

// 高级导航接口 (定距平动、定角转动)
class IAdvancedNavigation {
public:
    virtual bool movebydistance(double dist, double vx, double vy) = 0;
    virtual bool rotatebyangle(double angle, double vw) = 0;
    virtual ~IAdvancedNavigation() = default;
};

// 位置获取接口
class IPoseProvider {
public:
    virtual bool getpose(double& x, double& y, double& theta) = 0;
    virtual ~IPoseProvider() = default;
};

// ==========================================
// 抽象基类 (Base Class)
// ==========================================
class RobotMove 
{
public:
    RobotMove() = default;

    virtual bool init(const std::string& connection_str) = 0; // 初始化硬件连接
    virtual void setVelocity(double vx, double vy, double wz) = 0;
    virtual void send_loop() = 0;
    virtual void moveshutdown() = 0;

    virtual ~RobotMove() = default;
};

#endif // ROBOT_MOVE_BASE_HPP