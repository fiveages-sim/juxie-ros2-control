#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstdint>
#include <cmath>

namespace juxie_ros2_control {
class JxHardware : public hardware_interface::SystemInterface {
public:
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;
    
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;
    
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> 
        on_export_state_interfaces() override;
    
    std::vector<hardware_interface::CommandInterface::SharedPtr> 
        on_export_command_interfaces() override;
    
    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;
    
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    // 协议核心常量
    static constexpr uint32_t SYNC_FRAME_ID = 0x80;        // 同步帧ID
    static constexpr uint32_t MULTI_CTRL_ID = 0x200;       // 多电机控制帧ID
    static constexpr uint32_t SINGLE_FEEDBACK_BASE = 0x300;// 电机状态反馈基址
    static constexpr int CANFD_MAX_LEN = 64;               // CAN FD最大数据长度
    static constexpr double ANGLE_MIN = -180.0;            // 电机角度范围
    static constexpr double ANGLE_MAX = 180.0;
    static constexpr int16_t RAW_MIN = -32568;             // 协议定义的最小计数（对应-180度）
    static constexpr int16_t RAW_MAX = 32568;              // 协议定义的最大计数（对应180度）
    static constexpr uint8_t MAX_MOTOR_COUNT = 8;          // 多控帧最多支持8个电机
    bool is_first_command_ = true;  // 新增：标记是否是第一次发送命令

    // 仅保留必要配置参数
    std::string can_interface_;        // CAN接口名（默认can0）
    std::vector<uint8_t> motor_ids_;   // 电机ID列表（最多8个）
    int control_period_ms_;            // 控制周期（默认1ms）

    // 电机数据存储（仅位置+速度，移除effort）
    std::vector<double> joint_positions_;     // 实际位置（弧度）
    std::vector<double> joint_velocities_;    // 实际速度（弧度/秒）
    std::vector<double> joint_position_commands_; // 位置命令（弧度）
    std::vector<int16_t> raw_position_commands_;  // 位置命令（电机int16计数）

    // CAN FD通信相关
    int can_socket_;                    // CAN套接字
    struct ifreq ifr_;                  // 接口请求结构体
    struct sockaddr_can addr_;          // CAN地址结构体
    std::mutex state_mutex_;            // 状态数据互斥锁

    // 控制线程相关
    std::thread control_thread_;        // 周期性发送多控帧的线程
    std::atomic<bool> running_;         // 线程运行标志

    // 辅助函数（角度与电机计数映射，无减速比）
    int16_t angleToInt16(double angle); // 角度（°）→ 电机int16计数
    double int16ToAngle(int16_t int_val); // 电机int16计数 → 角度（°）
    bool initCanFd();                   // 初始化CAN FD接口（启用FD模式）
    bool enableMotors();                // 使能电机
    void controlLoop();                 // 控制线程循环
    bool readMotorStates();             // 读取所有电机状态
    bool sendMultiMotorCommand();       // 发送多控帧
    bool sendSyncFrame();               // 发送同步帧
};
} // namespace juxie_ros2_control