#pragma once
#include "juxie_ros2_control/gravity_feedforward_transition.h"
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
#include <chrono>
#include <cmath>
#include <unordered_map>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace juxie_ros2_control {
class JxHardware : public hardware_interface::SystemInterface {
public:
    ~JxHardware() override;
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;
    
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(
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
    enum class ControlMode
    {
        CSP_POSITION,   // Protocol mode 0x03
        CSV_VELOCITY,   // Protocol mode 0x04
        CURRENT_TORQUE, // Protocol mode 0x05; command is Iq in mA
    };

    // CAN-FD identifiers and control-word fields.
    static constexpr uint32_t SYNC_FRAME_ID = 0x80;
    static constexpr uint32_t MULTI_CTRL_ID = 0x200;
    static constexpr uint32_t SINGLE_FEEDBACK_BASE = 0x300;
    static constexpr int CANFD_MAX_LEN = 64;
    static constexpr double ANGLE_MIN = -180.0;  // Motor degrees
    static constexpr double ANGLE_MAX = 180.0;
    static constexpr int16_t RAW_MIN = -32668;  // Position counts at +/-180 degrees
    static constexpr int16_t RAW_MAX = 32668;
    static constexpr uint8_t CTRL_ENABLE = 0x80;
    static constexpr uint8_t CTRL_BRAKE_RELEASE = 0x40;
    static constexpr uint8_t CTRL_CLEAR_ERROR = 0x20;
    static constexpr uint8_t CTRL_MODE_CSP = 0x03 << 1;
    static constexpr uint8_t CTRL_MODE_CSV = 0x04 << 1;
    static constexpr uint8_t CTRL_MODE_CURRENT = 0x05 << 1;
    static constexpr uint8_t CTRL_WORD_CSP = CTRL_ENABLE | CTRL_BRAKE_RELEASE | CTRL_MODE_CSP;
    static constexpr uint8_t CTRL_WORD_CSP_CLEAR = CTRL_WORD_CSP | CTRL_CLEAR_ERROR;
    static constexpr uint8_t CTRL_WORD_CSV = CTRL_ENABLE | CTRL_BRAKE_RELEASE | CTRL_MODE_CSV;
    static constexpr uint8_t CTRL_WORD_CSV_CLEAR = CTRL_WORD_CSV | CTRL_CLEAR_ERROR;
    static constexpr uint8_t CTRL_WORD_CURRENT =
        CTRL_ENABLE | CTRL_BRAKE_RELEASE | CTRL_MODE_CURRENT;
    static constexpr uint8_t CTRL_WORD_CURRENT_CLEAR = CTRL_WORD_CURRENT | CTRL_CLEAR_ERROR;
    static_assert(CTRL_WORD_CSV == 0xC8, "CSV control word must match the Juxie protocol");
    static_assert(CTRL_WORD_CSV_CLEAR == 0xE8,
                  "CSV clear-error control word must match the Juxie protocol");
    static_assert(CTRL_WORD_CURRENT == 0xCA,
                  "Current-loop control word must match the Juxie protocol (mode 0x05)");
    static_assert(CTRL_WORD_CURRENT_CLEAR == 0xEA,
                  "Current-loop clear-error control word must match the Juxie protocol");
    static constexpr uint8_t MAX_MOTOR_COUNT = 8;

    std::string can_interface_;
    std::vector<uint8_t> motor_ids_;  // Same order as info_.joints
    std::vector<ControlMode> motor_control_modes_;
    std::vector<double> gear_ratios_;  // Motor speed / joint speed
    bool check_firmware_version_{true};
    float control_period_ms_;  // Sender thread period
    int32_t max_velocity_rpm_ = 32767;
    int32_t max_current_ma_ = 3000;  // Current command limit
    double current_slew_rate_ma_s_ = 0.0;  // 0 disables slew limiting

    // Current-loop gains and feedforward terms; currents are in mA.
    std::vector<double> impedance_kp_ma_rad_{};
    std::vector<double> impedance_kd_ma_s_rad_{};
    std::vector<double> friction_coulomb_ma_{};
    std::vector<double> friction_viscous_ma_s_rad_{};
    std::vector<double> friction_offset_ma_{};
    std::vector<double> gravity_feedforward_ma_{};

    // Required in current mode: body joints 1-3 index motors 1-4.
    std::string gravity_ff_table_path_;
    int gravity_ff_table_n_ = 0;  // Grid points per axis; 0 means unavailable
    double gravity_ff_table_lo_[3] = {0.0, 0.0, 0.0};
    double gravity_ff_table_hi_[3] = {1.0, 1.0, 1.0};
    std::vector<float> gravity_ff_table_data_;  // Flattened [i1][i2][i3][motor]
    std::vector<int> gravity_ff_table_slots_;
    bool loadGravityFfTable(const std::string& path);
    [[nodiscard]] bool gravityFeedforwardFromTable(std::vector<double>& out_ma) const;

    std::string gravity_ff_topic_;
    double gravity_ff_topic_timeout_s_ = 0.5;
    double gravity_ff_transition_s_ = 1.0;
    GravityFeedforwardTransition gravity_ff_transition_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_ff_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr dynamic_ff_sub_;
    mutable std::mutex gravity_ff_mutex_;
    std::vector<double> gravity_ff_topic_ma_;  // Indexed by motor ID - 1; values in mA
    mutable std::chrono::steady_clock::time_point gravity_ff_topic_stamp_{};
    static constexpr double dynamic_ff_timeout_s_ = 0.02;
    std::vector<double> dynamic_ff_topic_ma_;  // 同一 mutex 保护，按 motor slot 排列
    std::chrono::steady_clock::time_point dynamic_ff_topic_stamp_{};
    [[nodiscard]] bool gravityFeedforwardFromTopic(std::vector<double>& out_ma,
                                                  std::vector<double>* dynamic_ma = nullptr) const;

    // Latch to CSP hold after consecutive unhealthy read() cycles.
    // The gravity table couples waist motors 1-4; any current-mode waist enables the group interlock.
    bool waist_current_interlock_enabled_{false};
    std::atomic<bool> waist_current_fault_{false};
    std::vector<char> motor_ready_;
    // Caller holds state_mutex_. Never zero current as a fallback for a gravity-loaded axis.
    void tripWaistCurrentInterlock(const char* reason);
    [[nodiscard]] bool isWaistMotor(size_t index) const;
    std::vector<int> motor_unhealthy_count_;
    std::vector<char> motor_hold_latched_;
    int feedback_timeout_cycles_ = 10;

    // Command snapshots shared with the sender thread under state_mutex_.
    std::vector<double> position_reference_, velocity_reference_;
    std::vector<int16_t> latched_hold_positions_;
    std::atomic<bool> transmit_fault_{false};
    std::atomic<bool> gravity_ff_fault_{false};
    std::chrono::steady_clock::time_point transmit_retry_started_{};
    void resetPositionTarget(size_t index);

    // Caller must hold state_mutex_; reason is included in the error log.
    // 腰部承重链共用按“完整 4 轴”算出的电流，因此单轴锁存会被升级成整组锁存。
    void latchMotorToCspHold(size_t index, const char* reason);
    // 单轴实现，不升级成整组；只允许 tripWaistCurrentInterlock / latchMotorToCspHold 调用。
    void latchOneMotorToCspHold(size_t index, const char* reason);

    double friction_vel_eps_rad_s_ = 0.002;  // Friction feedforward deadband
    bool friction_ff_use_reference_vel_ = false;
    double friction_ff_max_ma_ = 0.0;  // 0 disables the limit
    int32_t max_position_step_nct_ = 25;  // Maximum CSP step per frame
    int32_t max_position_accel_nct_ = 5;  // 0 disables acceleration limiting

    // Joint state and command buffers.
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;  // Motor Iq feedback in mA
    std::vector<double> joint_position_commands_;
    std::vector<double> joint_velocity_commands_;
    std::vector<int16_t> raw_position_commands_;
    std::vector<int16_t> raw_velocity_commands_;
    std::vector<int16_t> raw_current_commands_;  // Iq commands in mA

    std::vector<int16_t> target_raw_position_commands_;
    std::vector<int16_t> last_sent_raw_commands_;
    std::vector<int32_t> last_sent_deltas_;
    std::vector<double> last_sent_current_commands_;

    bool is_first_command_ = true;
    std::atomic<bool> running_{false};

    int can_socket_ = -1;
    struct ifreq ifr_;
    struct sockaddr_can addr_;
    std::mutex state_mutex_;  // Protects shared command and state buffers

    std::thread control_thread_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr error_pub_;
    uint8_t locked_feedback_len_ = 0;

    int16_t angleToInt16(double angle);
    double int16ToAngle(int16_t int_val);
    bool hasVelocityControl() const;
    bool hasCurrentControl() const;
    const char* controlModeName() const;
    bool readFirmwareVersion(uint8_t motor_id, std::string& version);
    bool checkFirmwareVersions();
    bool initCanFd();
    void controlLoop();
    bool readMotorStates(bool strict_check);
    void parseAndReportErrorCode(uint8_t motor_id, uint16_t error_code_raw, const std::string& source);
    void parseAndReportEmergencyFrame(uint8_t motor_id, const canfd_frame& frame);
    bool sendMultiMotorCommand();
    bool sendSyncFrame();
};
}  // namespace juxie_ros2_control
