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
#include <chrono>
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
    // 12字节协议：下使能+抱闸（Bit7/Bit6 清零，保留 CSP 模式位）
    static constexpr uint8_t CTRL_WORD_CSP_DISABLE =
        CTRL_WORD_CSP & ~(CTRL_ENABLE | CTRL_BRAKE_RELEASE);
    // 7字节协议控制字
    static constexpr uint8_t CTRL_WORD_7_ENABLE = 0xD0;
    static constexpr uint8_t CTRL_WORD_7_ENABLE_CLEAR = 0xD8;
    static constexpr uint8_t CTRL_WORD_7_DISABLE = 0x10;   // 下使能+抱闸
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

    // === 空闲自动下使能/抱闸（可选功能）===
    // 是否启用空闲自动下使能。注意：必须在“命令确实长时间没变 + 已到位 + 静止”时才允许，
    // 否则慢速运动（每周期只变化 0~1 nct）会被误判为空闲，导致上下使能反复震荡并触发
    // 驱动“位置跃迁过大”故障。
    bool auto_disable_on_idle_ = true;
    // 距最后一次“目标变化”超过该时长才算空闲（ms）
    int32_t idle_timeout_ms_ = 500;
    // 到位判定容差（nct）：|反馈 - 目标| 需小于该值
    int32_t idle_position_tol_nct_ = 2;
    // 静止判定容差（rad/s）：|反馈速度| 需小于该值
    double idle_velocity_tol_ = 0.02;
    // 下使能状态下重新上使能所需的最小命令变化（nct），用于抑制量化抖动
    int32_t enable_deadband_nct_ = 10;

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
    bool is_first_command_ = true;    // 标记是否是第一次发送命令（含重新上使能首帧）
    std::atomic<bool> running_;       // 线程运行标志
    // true 表示处于“上使能跟踪”状态，控制线程会持续下发 CSP 控制帧
    std::atomic<bool> command_pending_{false};
    // 满足空闲条件（超时+到位+静止）：发送一次下使能/抱闸帧后停发
    std::atomic<bool> disable_pending_{false};
    // 是否已处于下使能状态（由 state_mutex_ 保护）
    bool motors_disabled_ = false;
    // 最后一次目标发生变化的时刻（用于空闲超时判定，由 state_mutex_ 保护）
    std::chrono::steady_clock::time_point last_command_change_time_;
    // 下使能时刻的位置，用于上使能死区判断（由 state_mutex_ 保护）
    std::vector<int16_t> disabled_hold_raw_;

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
    int16_t angleToInt16(double angle) const; // 角度（°）→ 电机int16计数
    double int16ToAngle(int16_t int_val); // 电机int16计数 → 角度（°）
    bool initCanFd();                   // 初始化CAN FD接口（启用FD模式）
    bool enableMotors();                // 使能电机
    void controlLoop();                 // 控制线程循环
    bool readMotorStates(bool strict_check); // 读取所有电机状态
    void parseAndReportErrorCode(uint8_t motor_id, uint16_t error_code_raw, const std::string& source);
    void parseAndReportEmergencyFrame(uint8_t motor_id, const canfd_frame& frame);
    bool sendMultiMotorCommand();       // 发送多控使能帧
    bool sendDisableMultiMotorCommand(); // 发送下使能/抱闸多控帧
    // 空闲判定：已到位、速度为 0、且无位置跟踪误差
    bool isIdleAtTarget() const;
    bool sendSyncFrame();               // 发送同步帧
    uint8_t getEnableControlByte() const;
    uint8_t getDisableControlByte() const;
    // 用当前反馈位置对齐下发命令，避免上下使能后首帧位置跃迁
    void syncCommandsFromFeedback();
};
} // namespace juxie_ros2_control