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
#include <unordered_map>
#include <std_msgs/msg/int64.hpp>

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
    // === 新协议适配 START: 关节模组使用协议简易说明书 V1.0 ===
    static constexpr int16_t RAW_MIN = -32668;             // CSP位置范围下限（对应-180度）
    static constexpr int16_t RAW_MAX = 32668;              // CSP位置范围上限（对应180度）
    static constexpr uint8_t CTRL_ENABLE = 0x80;           // Bit[7] 上使能
    static constexpr uint8_t CTRL_BRAKE_RELEASE = 0x40;    // Bit[6] 抱闸释放
    static constexpr uint8_t CTRL_CLEAR_ERROR = 0x20;      // Bit[5] 复位错误
    static constexpr uint8_t CTRL_MODE_CSP = 0x03 << 1;    // Bit[4:1] CSP循环同步位置模式
    static constexpr uint8_t CTRL_WORD_CSP = CTRL_ENABLE | CTRL_BRAKE_RELEASE | CTRL_MODE_CSP;
    static constexpr uint8_t CTRL_WORD_CSP_CLEAR = CTRL_WORD_CSP | CTRL_CLEAR_ERROR;
    // === 新协议适配 END ===
    static constexpr uint8_t MAX_MOTOR_COUNT = 8;          // 多控帧最多支持8个电机

    // 配置参数
    std::string can_interface_;        // CAN接口名（默认can0）
    std::vector<uint8_t> motor_ids_;   // 电机ID列表（最多8个）
    float control_period_ms_;            // 控制周期（默认1ms）
    // CSP 单帧最大位置增量（nct）。文档写 1ms 下 >100 报错；实测 2ms 下约 100+ 也会报，
    // 按每帧绝对值限幅，默认 90。
    int32_t max_position_step_nct_ = 25;
    // 相邻两帧指令速度(delta)的最大变化量（nct/cycle²）；0 表示关闭加速度限幅。
    int32_t max_position_accel_nct_ = 5;

    // 状态和命令存储
    std::vector<double> joint_positions_;     // 实际位置（弧度）
    std::vector<double> joint_velocities_;    // 实际速度（弧度/秒）
    std::vector<double> joint_efforts_;       // 实际力矩
    std::vector<double> joint_position_commands_; // 位置命令（弧度）
    std::vector<int16_t> raw_position_commands_;  // 位置命令（电机int16计数）

    // 限速/限加速追赶：始终朝最新目标追
    std::vector<int16_t> target_raw_position_commands_; // 最新目标（来自 ROS2 控制器）
    std::vector<int16_t> last_sent_raw_commands_;       // 上一次实际发送的命令
    std::vector<int32_t> last_sent_deltas_;             // 上一次实际发送的位置增量（速度）

    // 控制标志
    bool is_first_command_ = true;    // 标记是否是第一次发送命令
    std::atomic<bool> running_;       // 线程运行标志

    // CAN FD通信相关
    int can_socket_;                    // CAN套接字
    struct ifreq ifr_;                  // 接口请求结构体
    struct sockaddr_can addr_;          // CAN地址结构体
    std::mutex state_mutex_;            // 状态数据互斥锁

    // 控制线程相关
    std::thread control_thread_;        // 周期性发送多控帧的线程
    // 异常发布相关
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr error_pub_;
    // 在严格检查阶段首次锁定反馈帧长度，后续沿用该长度做校验
    uint8_t locked_feedback_len_ = 0;

    // 辅助函数
    int16_t angleToInt16(double angle); // 角度（°）→ 电机int16计数
    double int16ToAngle(int16_t int_val); // 电机int16计数 → 角度（°）
    bool initCanFd();                   // 初始化CAN FD接口（启用FD模式）
    bool enableMotors();                // 使能电机
    void controlLoop();                 // 控制线程循环
    bool readMotorStates(bool strict_check); // 读取所有电机状态
    void parseAndReportErrorCode(uint8_t motor_id, uint16_t error_code_raw, const std::string& source);
    void parseAndReportEmergencyFrame(uint8_t motor_id, const canfd_frame& frame);
    bool sendMultiMotorCommand();       // 发送多控帧
    bool sendSyncFrame();               // 发送同步帧
};
} // namespace juxie_ros2_control