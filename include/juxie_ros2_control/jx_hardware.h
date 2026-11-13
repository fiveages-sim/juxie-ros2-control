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
#include <std_msgs/msg/int32_multi_array.hpp>

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
    // 严重故障兜底：**失能 + 不释放抱闸**（只留 CSP 模式位 ⇒ 0x06）。
    // 抱闸是弹簧/失电型，抱死不需要母线电流 —— 欠压时这是唯一还能托住重力的动作。
    // ⚠️ 绝不能带 CTRL_ENABLE / CTRL_BRAKE_RELEASE，否则等于又把电机使能 + 松闸。
    static constexpr uint8_t CTRL_WORD_BRAKE_ENGAGE = CTRL_MODE_CSP;
    static_assert(CTRL_WORD_BRAKE_ENGAGE == 0x06,
                  "Brake-engage control word must be mode-CSP only (enable/brake bits clear)");
    // 带清错位的抱闸字（0x26）：驱动器把欠压锁成故障态后，**不带清错位的
    // 控制字可能被直接丢弃**，于是抱闸永远不动作，重力直接把人送下去。
    // 清错位不会让电机重新使能（enable / brake-release 位仍为 0），
    // 只是让驱动器肯接受这一帧。
    static constexpr uint8_t CTRL_WORD_BRAKE_ENGAGE_CLEAR =
        CTRL_WORD_BRAKE_ENGAGE | CTRL_CLEAR_ERROR;  // 0x26
    static_assert(CTRL_WORD_BRAKE_ENGAGE_CLEAR == 0x26,
                  "Brake-engage clear word must keep enable/brake-release clear");
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
    // 静摩擦 f_s（> f_c 时生效）与 Stribeck 速度 v_s：关节卡住时补 f_s，滑起来后只剩 f_c。
    // 不配（默认全 0 / v_s=0）= 关闭，行为与之前逐字节一致。
    std::vector<double> friction_stiction_ma_{};
    double stribeck_vel_rad_s_ = 0.0;
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
    // 严重故障（欠压/过压/堵转…）后置位：该轴此后每帧都发“失能+抱闸”控制字。
    // 与 motor_hold_latched_ 一样只在 on_init 清 —— 故障通道不因重新激活自动变可信。
    std::vector<char> brake_engage_requested_;
    bool brake_on_waist_fault_{true};  // 腰部故障时整组抱闸（默认开）
    // on_deactivate 里持续发抱闸字的时长。只发 3 帧（30 ms）时，若驱动器
    // 在故障态下丢掉前几帧，抱闸就永远没合上。留 500 ms 给抱闸线圈失电 + 机械咬合。
    int brake_hold_ms_{500};
    int feedback_timeout_cycles_ = 10;

    // ── 起动电流辨识（ident）──────────────────────────────────────────────
    // 为什么需要：CST 摩擦前馈里的 f_s 只由最低速分箱约束，与 f_off / v_s 强共线，
    // 用闭环数据每轮重拟合会漂几百 mA（2026-11-03 实测 +15%~+40%）。这里换成
    // **开环电流 ramp + 编码器跳变**：电流一点点加，直到该轴真的动了，那个电流
    // 就是「该姿态下需要多少才动」，完全不依赖 Kp / Kd / 重力模型 / URDF 零点。
    //
    // 关键设计：**保留重力前馈**（current = g_ff + ramp）。
    //   · 不保留的话带载轴（j2 挂着 5517 mA）会直接掉下来；
    //   · 保留的话 ramp 要克服的只剩纯摩擦 ⇒ 测出来直接是 f_s + f_off；
    //   · 而且带载轴也能测，不只偏航轴。
    //
    // 安全：默认关；只让**一个**轴进 ident，其余轴照常跑阻抗；ramp 上限、超时、
    //       行程超限任一触发就放弃；任何退出路径都回到正常阻抗（含 CSP 保持）。
    bool ident_enable_{false};
    double ident_max_ma_{3000.0};
    double ident_ramp_ma_s_{200.0};
    double ident_move_rad_{2.88e-4};   // 默认 3 个编码器 LSB（0.0055° ⇒ 9.6e-5 rad）
    double ident_timeout_s_{5.0};
    enum class IdentPhase { IDLE, RAMP };
    // 跨线程：订阅回调（ROS 线程）写，sendMultiMotorCommand（发送线程）读写。
    // 用 atomic 保证「回调把 base_q/sign 写完」对发送线程是可见的，并且
    // 发送线程不会在回调改到一半时读到半新半旧的状态。
    std::atomic<IdentPhase> ident_phase_{IdentPhase::IDLE};
    size_t ident_motor_index_{0};
    int ident_sign_{0};
    double ident_base_q_{0.0};
    double ident_ramp_ma_{0.0};
    double ident_elapsed_s_{0.0};
    double ident_frozen_gravity_ma_{0.0};
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr ident_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ident_result_pub_;
    void startIdent(size_t index, int sign);
    void finishIdent(bool ok, const char* why);
    void advanceIdent(size_t index, double dt_s, double gravity_ma);

    // ── 逐项分解调试通道 ──────────────────────────────────────────────────
    // 全身模式下 q_d 不可观测（WBC 直写 command interface），所以从驱动内部把
    // 阻抗律的每一项拆开发出去。有订阅者才发，没人订阅零开销。
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr impedance_debug_pub_;
    int impedance_debug_decim_{5};
    uint32_t impedance_debug_counter_{0};

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
    // 严重故障：把该轴切到“失能 + 抱闸”，此后一直保持（CSP 请求在欠压时出不了力）。
    void requestBrakeEngage(size_t index, const char* reason);

    double friction_vel_eps_rad_s_ = 0.002;  // Friction feedforward deadband
    bool friction_ff_use_reference_vel_ = false;
    bool friction_ff_fill_deadband_ = false;  // friction_ff_velocity=reference_ramp
    // 死区内补摩擦的斜坡宽度（0 = 沿用 friction_vel_eps_rad_s）。死区是“用哪个速度判
    // 方向”，斜坡是“补多少”，两个门槛分开才能让低速升降三轴都补满。
    double friction_ff_ramp_vel_rad_s_ = 0.0;
    double friction_ff_max_ma_ = 0.0;  // 0 disables the limit
    // 2026-11-11：摩擦前馈用的**参考速度**先做一阶低通，再加噪声死区。
    // 起因：全身控制下 WBC 的 v_d 在 0 附近抖 ±0.01 rad/s（≈11 Hz），
    // 而 reference_ramp 分支的 scale = |v_d|/ramp（ramp=0.0015）一饱和就是满幅，
    // 于是 ±f_s 跟着噪声翻号 —— 实测 Σd·ff 峰峰 2357 mA ≈ ±40 N·m，位置不动。
    // 低通把 ≈11 Hz 的抖动压到 ~2 Hz 以下，min_vel 再把残余卡掉。
    // 两者默认 0 = 关闭，行为与改动前完全一致。
    double friction_ff_vd_filter_hz_ = 0.0;   // 0 = 不滤波
    double friction_ff_min_vel_rad_s_ = 0.0;  // 0 = 不设噪声死区
    std::vector<double> ff_vd_filt_;          // 每轴低通后的 v_d
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
    bool readMotorStates(bool strict_check, bool enforce_waist_interlock = true);
    // 腰部四轴是否全部“已使能 + 已松闸 + 反馈正常”（供 on_activate 冷启动使能后复查）。
    bool waistGroupReady();
    // 冷启动时模块通常未使能：先发 CSP 保持帧（目标=实测位置）把它使能，再进电流环。
    bool enableWaistInCspHold();
    void parseAndReportErrorCode(uint8_t motor_id, uint16_t error_code_raw, const std::string& source);
    // 同一电机的错误码只在**变化时**或每 1 s 才上报一次。
    // 电机持续报错时 feedback 帧每帧都会带错误位（500 Hz）；不节流就会在读路径里
    // 每秒刷 500×(错误位个数) 条 INFO + ERROR，实测 55 min 刷出 518 MB 日志，
    // 并把 controller_manager 的 2 ms 周期撑到 7.4 ms（Read time 731 us，
    // 2026-11-03 W2 实测）—— 表现为升降时机身前后抖、阻抗静态精度变差。
    std::vector<uint16_t> last_reported_error_code_;  // 按 motor id 索引
    std::vector<double> last_error_report_s_;         // 上次上报时刻 [s]
    std::vector<uint32_t> suppressed_error_reports_;  // 期间被抑制的帧数
    void parseAndReportEmergencyFrame(uint8_t motor_id, const canfd_frame& frame);
    bool sendMultiMotorCommand();
    bool sendSyncFrame();
};
}  // namespace juxie_ros2_control
