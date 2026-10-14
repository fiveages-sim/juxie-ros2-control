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
#include <chrono>
#include <cmath>
#include <fstream>
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
        CSP_POSITION,
        CSV_VELOCITY,
        // 电流环模式（协议 0x05）：帧内目标值 = 电机目标 Iq 电流，单位 **mA**。
        // 模块**不做斜坡、不限速、不保持位置**：限幅/限速率都在驱动这层做。
        CURRENT_TORQUE,
    };

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
    static constexpr uint8_t CTRL_MODE_CSV = 0x04 << 1;    // Bit[4:1] CSV循环同步速度模式
    static constexpr uint8_t CTRL_MODE_CURRENT = 0x05 << 1;// Bit[4:1] 电流环模式（目标 Iq，单位 mA）
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
    // === 新协议适配 END ===
    static constexpr uint8_t MAX_MOTOR_COUNT = 8;          // 多控帧最多支持8个电机

    // 配置参数
    std::string can_interface_;        // CAN接口名（默认can0）
    std::vector<uint8_t> motor_ids_;   // 电机ID列表（最多8个）
    /// 每个电机的模式（on_init 从参数定下）。**有两处会改写它**：
    ///   ① `latchMotorToCspHold()`：反馈通道不可信 ⇒ 改成 CSP 并**不再恢复**，直到重新
    ///      on_init（锁存状态另记在 motor_hold_latched_ 里，write() 靠它忽略上层命令）；
    ///   ② `on_deactivate()`：把全部电机临时改成 CSP 发几帧切换帧，发完恢复。
    std::vector<ControlMode> motor_control_modes_;
    std::vector<double> gear_ratios_;  // 电机端转速 / 输出端转速
    bool check_firmware_version_{true};
    // **发帧周期**：控制线程按它周期性地调 sendMultiMotorCommand()（当前 xacro 配的是
    // 2 ms = 500 Hz）。与 CM 的 update_rate **无关**，所以改 CM 频率不影响腰部阻抗环的节拍。
    float control_period_ms_;
    int32_t max_velocity_rpm_ = 32767;
    // 电流环模式的命令限幅（mA）与变化率限制（mA/s，0 = 关闭）。
    // 模块自身不平滑电流，所以变化率限制必须在这里做，否则力矩会阶跃上去。
    int32_t max_current_ma_ = 3000;
    // ⚠️ 配了重力前馈（gravity_ff_table）时**不要**开这个：前馈必须在第一帧就建立（它是
    //    抵抗重力的稳态力矩），限速会把它拖成几百 ms 的斜坡，这段里承重轴会下沉。
    double current_slew_rate_ma_s_ = 0.0;

    // ── 底层阻抗层（单位全部是 mA，与反馈 Iq 同量纲 ⇒ 不需要 Kt）──────────────
    //   I_cmd = Kp·(q_d − q) + Kd·(q̇_d − q̇) + g_ff + f_off
    //           + (|v_ff| < v_eps ? 0 : sign(v_ff)·(f_c + f_v·|v_ff|))
    //   v_ff 默认取 q̇；friction_ff_use_reference_vel_ 为真时，|q̇_d| >= v_eps 就取 q̇_d。
    //   前馈幅值再受 friction_ff_max_ma_ 限制（0 = 不限）。
    // q_d / q̇_d 取自**上层的 position / velocity 命令口**（OCS2 照旧只发位置/速度），
    // 所以上层设计不变；换成电流是本驱动在**独立控制线程**里做的，节奏 = control_period_ms_
    // （当前 2 ms = 500 Hz），与 CM 的 update_rate 无关。
    // 列表长度 = 电机个数（与 gear_ratios 同一套解析：1 个值 = 全部同值）。
    std::vector<double> impedance_kp_ma_rad_{};        // mA/rad     弹簧
    std::vector<double> impedance_kd_ma_s_rad_{};      // mA·s/rad   阻尼
    std::vector<double> friction_coulomb_ma_{};        // mA         库仑
    std::vector<double> friction_viscous_ma_s_rad_{};  // mA·s/rad   粘性
    std::vector<double> friction_offset_ma_{};         // mA         偏置（可为负）
    std::vector<double> gravity_feedforward_ma_{};     // mA         重力前馈（可为负）

    // ── 腰部重力前馈查找表：**不依赖任何控制器**，是无控制器时唯一能把腰顶住的东西 ──
    // 背景：ocs2_wbc_controller 必须同时 claim 20 个关节的命令口，只要手臂（M6_CCS）出错
    //   它就激活不了；而驱动照样会把腰按位置前馈切进电流环 ⇒ 没人补重力 ⇒ 腰 1/2/3 只剩
    //   弹簧、会下沉几度（2026-10-13 真机实测）。把模型放进驱动是唯一不依赖上层的位置。
    // 数据：τ_g(body_joint1~3) 在 (q1,q2,q3) 网格上的 mA 值，手臂/头部固定在标称姿态
    //   （实测该近似最坏 0.86°；忽略 q4 最坏 0.86°；13³ 网格插值 0.32°）。
    //   由 scripts/gen_waist_gravity_table.py 离线生成。
    // 用法：每个控制周期从**实测**腰部角度三线性插值，加到 gravity_feedforward_ma_ 上。
    //   路径留空 = 关闭（行为与从前完全一致）。
    std::string gravity_ff_table_path_;
    int gravity_ff_table_n_ = 0;                        // 每轴点数；0 = 表不可用
    double gravity_ff_table_lo_[3] = {0.0, 0.0, 0.0};
    double gravity_ff_table_hi_[3] = {1.0, 1.0, 1.0};
    /// 扁平存 [i1][i2][i3][col]，col = 0..3 对应电机 ID 1,2,3,4
    std::vector<float> gravity_ff_table_data_;
    /// 表里那 4 列分别对应本驱动的哪个电机下标（按电机 ID 1,2,3,4 顺序）
    std::vector<int> gravity_ff_table_slots_;
    /// 从文件加载重力前馈表；返回 false 表示失败（调用方决定是否报错退出）
    bool loadGravityFfTable(const std::string& path);
    /// 用当前实测腰部角度插值出每个电机的重力前馈（mA，长度 = 电机个数）；表不可用则返回 false
    [[nodiscard]] bool gravityFeedforwardFromTable(std::vector<double>& out_ma) const;

    // ── 上层（控制器）推送的【精确】重力前馈（含真实手臂/头部姿态）────────────────
    // 两层分工：
    //   查表      = **保命层**：不依赖任何控制器，手臂挂了、WBC 起不来也照旧把腰顶住（~1°）。
    //   本话题    = **精度层**：ocs2_wbc_controller 活着时用它自己的 20 关节模型 +
    //               **真实手臂角度** 算 τ_g（离线已验证：手臂姿态的影响最大 0.86°）。
    // 驱动**新鲜就用话题值、过期就退回查表**，不叠加 ⇒ 不会重复补偿。
    // 话题布局：Float64MultiArray，长度 = 6，**下标 = 电机ID−1**（值 = mA，可为负）。
    std::string gravity_ff_topic_;
    double gravity_ff_topic_timeout_s_ = 0.5;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_ff_sub_;
    mutable std::mutex gravity_ff_mutex_;
    std::vector<double> gravity_ff_topic_ma_;                 // 按 motor_ids_ 下标；空 = 没收到
    /// 用 steady_clock 而不是 ROS 时间：不依赖 /clock（use_sim_time 下也安全）
    mutable std::chrono::steady_clock::time_point gravity_ff_topic_stamp_{};
    /// 取话题值（按本驱动的电机下标）；过期/未收到则返回 false
    [[nodiscard]] bool gravityFeedforwardFromTopic(std::vector<double>& out_ma) const;

    // ── 逐电机反馈健康度：防止「拿旧数据继续算电流」 ──────────────────────────
    // 为什么必要：反馈一旦中断，joint_positions_/joint_velocities_ 会停在旧值，而阻抗律
    //   照旧按旧值算 ⇒`Kp·(q_d − q_旧)` 会一直涨到限幅（实测可到 17 A），而驱动完全不
    //   知道关节在哪；`read()` 还返回 OK，CM 也不会停硬件。
    // 处理：每拍 read() 对每个电机检查 —— 没回帧、或自己报「未使能」就累加计数，收到正常
    //   帧则清零；连续超过 feedback_timeout_cycles_ 就**把该电机锁存成 CSP 保持**（模块用
    //   自己的位置环抱住最后已知位置），并放弃上层对它的命令。
    // ⚠️ **绝不把电流置 0**：那对重力关节等于自由落体。
    // ⚠️ 锁存**不会被 on_activate 清除**（只由 on_init 清）—— 「这条通道不可信」不会因为
    //    一次重新激活就变可信。清掉它只会让电流阻抗轴静默变成 CSP 轴。
    std::vector<int> motor_unhealthy_count_;
    std::vector<char> motor_hold_latched_;
    /// 连续多少拍不可用就锁存（CM 500 Hz → 10 拍 = 20 ms）
    int feedback_timeout_cycles_ = 10;

    // Only write() reads controller command interfaces. The sender reads these
    // validated snapshots under state_mutex_.
    std::vector<double> position_reference_, velocity_reference_;
    std::vector<int16_t> latched_hold_positions_;
    std::atomic<bool> transmit_fault_{false};
    /// 首次遇到可重试发送失败的时刻（epoch 为 0 表示当前不在重试窗口内）
    std::chrono::steady_clock::time_point transmit_retry_started_{};
    void resetPositionTarget(size_t index);

    /**
     * 把某个电机锁存成「CSP 保持最后一帧位置」。
     * ⚠️ 调用方必须已持有 state_mutex_（readMotorStates() 内部调用）。
     * @param reason 锁存原因，会写进 ERROR 日志（必填，免得日志里出现空原因）
     */
    void latchMotorToCspHold(size_t index, const char* reason);

    double friction_vel_eps_rad_s_ = 0.002;            // 摩擦符号的速度死区（防静止自激）
    // 摩擦前馈的符号取哪个速度：
    //   false（默认）＝ 实测速度。被外力推动时前馈照样对抗摩擦，但**换向时慢一步**
    //                  （要等轴真的反向，前馈才翻号）。
    //   true          ＝ 参考速度（仅当 |v_d| >= v_eps，否则退回实测速度）。
    //   真机实测（2026-10-12，三角波 ±4°/0.05 rad/s，Kp=20000、Kd=6900）：
    //   跟踪 RMS 0.223° → 0.093°，最大误差 0.625° → 0.255°，电流基本不变。
    bool friction_ff_use_reference_vel_ = false;
    // 摩擦前馈幅值上限（mA）；0 = 不限幅。
    // 前馈 = f_c + f_v·|v|，现场 f_v=6300 mA·s/rad ⇒ |v| 到 (max_ma−f_c)/f_v ≈ 0.19 rad/s
    // 就把电流顶满；而 f_v 只在 0.04 rad/s 以内辨识过（再高属外推）⇒ 限一下更安全。
    double friction_ff_max_ma_ = 0.0;
    // CSP 单帧最大位置增量（nct）。文档写 1ms 下 >100 报错；实测 2ms 下约 100+ 也会报，
    // 按每帧绝对值限幅，默认 90。
    int32_t max_position_step_nct_ = 25;
    // 相邻两帧指令速度(delta)的最大变化量（nct/cycle²）；0 表示关闭加速度限幅。
    int32_t max_position_accel_nct_ = 5;

    // 状态和命令存储
    std::vector<double> joint_positions_;     // 实际位置（弧度）
    std::vector<double> joint_velocities_;    // 实际速度（弧度/秒）
    std::vector<double> joint_efforts_;       // 实际力矩（新协议里就是 Iq，mA）
    std::vector<double> joint_position_commands_; // 位置命令（弧度）
    std::vector<double> joint_velocity_commands_; // 速度命令/参考（弧度/秒）
    std::vector<int16_t> raw_position_commands_;  // 位置命令（电机int16计数）
    std::vector<int16_t> raw_velocity_commands_;  // CSV电机端速度（RPM）
    std::vector<int16_t> raw_current_commands_;   // 电流环：下发值（mA，int16）

    // 限速/限加速追赶：始终朝最新目标追
    std::vector<int16_t> target_raw_position_commands_; // 最新目标（来自 ROS2 控制器）
    std::vector<int16_t> last_sent_raw_commands_;       // 上一次实际发送的命令
    std::vector<int32_t> last_sent_deltas_;             // 上一次实际发送的位置增量（速度）
    std::vector<double> last_sent_current_commands_;    // 上一次实际发送的电流命令（mA，变化率限制用）

    // 控制标志
    bool is_first_command_ = true;    // 标记是否是第一次发送命令
    // 控制线程的运行标志。readMotorStates() 发现电机报错时也会置 false ⇒ 控制线程立刻
    // 停止发帧（模块保持最后一个电流命令）。写它的是 CM 更新线程 / 生命周期回调线程，
    // 读它的是控制线程 ⇒ 必须是 atomic。
    std::atomic<bool> running_{false};

    // CAN FD通信相关
    int can_socket_ = -1;               // CAN套接字
    struct ifreq ifr_;                  // 接口请求结构体
    struct sockaddr_can addr_;          // CAN地址结构体
    std::mutex state_mutex_;            // 状态数据互斥锁

    // 控制线程相关
    std::thread control_thread_;        // 周期性发送多控帧（周期 = control_period_ms_）
    // 异常发布相关
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr error_pub_;
    // 在严格检查阶段首次锁定反馈帧长度，后续沿用该长度做校验
    uint8_t locked_feedback_len_ = 0;

    // 辅助函数
    int16_t angleToInt16(double angle); // 角度（°）→ 电机int16计数
    double int16ToAngle(int16_t int_val); // 电机int16计数 → 角度（°）
    bool hasVelocityControl() const;
    bool hasCurrentControl() const;
    const char* controlModeName() const;
    bool readFirmwareVersion(uint8_t motor_id, std::string& version);
    bool checkFirmwareVersions();
    bool initCanFd();                   // 初始化CAN FD接口（启用FD模式）
    void controlLoop();                 // 控制线程循环（固定 control_period_ms_ 节奏发帧）
    bool readMotorStates(bool strict_check); // 读取所有电机状态
    void parseAndReportErrorCode(uint8_t motor_id, uint16_t error_code_raw, const std::string& source);
    void parseAndReportEmergencyFrame(uint8_t motor_id, const canfd_frame& frame);
    bool sendMultiMotorCommand();       // 算电流 + 打包 + 发 0x200 多控帧
    bool sendSyncFrame();               // 发送同步帧
};
} // namespace juxie_ros2_control
