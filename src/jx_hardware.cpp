#include "juxie_ros2_control/jx_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <cctype>
#include <fstream>
#include <regex>
#include <fcntl.h>
#include <poll.h>

namespace juxie_ros2_control {

JxHardware::~JxHardware() {
    if (can_socket_ >= 0) on_deactivate(rclcpp_lifecycle::State{});
}

hardware_interface::CallbackReturn JxHardware::on_error(const rclcpp_lifecycle::State& state) {
    return on_deactivate(state);
}

hardware_interface::CallbackReturn JxHardware::on_shutdown(const rclcpp_lifecycle::State& state) {
    return on_deactivate(state);
}

bool JxHardware::hasVelocityControl() const {
    return std::find(motor_control_modes_.begin(), motor_control_modes_.end(),
                     ControlMode::CSV_VELOCITY) != motor_control_modes_.end();
}

bool JxHardware::hasCurrentControl() const {
    return std::find(motor_control_modes_.begin(), motor_control_modes_.end(),
                     ControlMode::CURRENT_TORQUE) != motor_control_modes_.end();
}

const char* JxHardware::controlModeName() const {
    bool has_csv = false;
    bool has_csp = false;
    bool has_current = false;
    for (const auto mode : motor_control_modes_) {
        has_csv = has_csv || mode == ControlMode::CSV_VELOCITY;
        has_csp = has_csp || mode == ControlMode::CSP_POSITION;
        has_current = has_current || mode == ControlMode::CURRENT_TORQUE;
    }
    const int kinds = static_cast<int>(has_csv) + static_cast<int>(has_csp) + static_cast<int>(has_current);
    if (kinds > 1) {
        return "mixed CSP/CSV/CURRENT";
    }
    if (has_current) {
        return "current loop (torque, mA)";
    }
    return has_csv ? "CSV velocity" : "CSP position";
}

bool JxHardware::readFirmwareVersion(uint8_t motor_id, std::string& version) {
    constexpr auto response_timeout = std::chrono::milliseconds(200);
    constexpr size_t max_version_length = 64;
    const canid_t request_id = static_cast<canid_t>(0x600 + motor_id);
    const canid_t response_id = static_cast<canid_t>(0x580 + motor_id);
    version.clear();

    const auto send_request = [&](uint8_t command, bool initiate) {
        can_frame request{};
        request.can_id = request_id;
        request.can_dlc = 8;
        request.data[0] = command;
        if (initiate) {
            request.data[1] = 0x0A;  // 0x100A:00 firmware version
            request.data[2] = 0x10;
            request.data[3] = 0x00;
        }
        if (::write(can_socket_, &request, sizeof(request)) != sizeof(request)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Failed to send firmware SDO request to motor %u: %s",
                         motor_id, strerror(errno));
            return false;
        }
        return true;
    };

    const auto wait_response = [&](canfd_frame& response) {
        const auto deadline = std::chrono::steady_clock::now() + response_timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            const ssize_t bytes = ::read(can_socket_, &response, sizeof(response));
            if (bytes > 0) {
                if ((response.can_id & CAN_SFF_MASK) != response_id || response.len < 8) {
                    continue;
                }
                if (response.data[0] == 0x80) {
                    const uint32_t abort_code = static_cast<uint32_t>(response.data[4]) |
                                                (static_cast<uint32_t>(response.data[5]) << 8) |
                                                (static_cast<uint32_t>(response.data[6]) << 16) |
                                                (static_cast<uint32_t>(response.data[7]) << 24);
                    RCLCPP_ERROR(get_node()->get_logger(),
                                 "Motor %u aborted firmware SDO read: 0x%08X",
                                 motor_id, abort_code);
                    return false;
                }
                return true;
            }
            if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Firmware SDO read failed for motor %u: %s",
                             motor_id, strerror(errno));
                return false;
            }
            pollfd pfd{};
            pfd.fd = can_socket_;
            pfd.events = POLLIN;
            (void)::poll(&pfd, 1, 2);
        }
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Timed out reading firmware version from motor %u", motor_id);
        return false;
    };

    if (!send_request(0x40, true)) {
        return false;
    }
    canfd_frame response{};
    if (!wait_response(response)) {
        return false;
    }

    const uint8_t command = response.data[0];
    if ((command & 0xE0) != 0x40 || response.data[1] != 0x0A ||
        response.data[2] != 0x10 || response.data[3] != 0x00) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Invalid firmware SDO initiate response from motor %u", motor_id);
        return false;
    }

    if ((command & 0x02) != 0) {
        const size_t count = (command & 0x01) != 0 ? 4U - ((command >> 2) & 0x03) : 4U;
        version.assign(reinterpret_cast<const char*>(&response.data[4]), count);
    } else {
        const bool size_known = (command & 0x01) != 0;
        size_t expected_size = max_version_length;
        if ((command & 0x01) != 0) {
            expected_size = static_cast<size_t>(response.data[4]) |
                            (static_cast<size_t>(response.data[5]) << 8) |
                            (static_cast<size_t>(response.data[6]) << 16) |
                            (static_cast<size_t>(response.data[7]) << 24);
            if (expected_size == 0 || expected_size > max_version_length) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Motor %u reported an invalid firmware string length: %zu",
                             motor_id, expected_size);
                return false;
            }
        }

        uint8_t toggle = 0;
        bool complete = false;
        while (version.size() < expected_size) {
            if (!send_request(static_cast<uint8_t>(0x60 | (toggle << 4)), false) ||
                !wait_response(response)) {
                return false;
            }
            const uint8_t segment_command = response.data[0];
            if ((segment_command & 0xE0) != 0x00 ||
                ((segment_command >> 4) & 0x01) != toggle) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Invalid firmware SDO segment response from motor %u", motor_id);
                return false;
            }
            const size_t count = 7U - ((segment_command >> 1) & 0x07);
            if (!(segment_command & 0x01) && count != 7U) return false;
            version.append(reinterpret_cast<const char*>(&response.data[1]), count);
            if (version.size() > max_version_length) {
                return false;
            }
            if ((segment_command & 0x01) != 0) {
                complete = true;
                break;
            }
            toggle ^= 1U;
        }
        if (!complete || (size_known && version.size() != expected_size)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Incomplete firmware SDO value from motor %u", motor_id);
            return false;
        }
    }

    while (!version.empty() && (version.back() == '\0' || std::isspace(
               static_cast<unsigned char>(version.back())))) {
        version.pop_back();
    }
    return !version.empty();
}

bool JxHardware::checkFirmwareVersions() {
    constexpr int required_major = 1;
    constexpr int required_minor = 16;
    constexpr int required_patch = 7;
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (motor_control_modes_[i] != ControlMode::CSV_VELOCITY) {
            continue;
        }
        std::string version;
        if (!readFirmwareVersion(motor_ids_[i], version)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Cannot verify firmware version for motor %u; activation aborted",
                         motor_ids_[i]);
            return false;
        }
        const std::regex pattern(R"(^\s*[vV]?([0-9]{1,4})\.([0-9]{1,4})\.([0-9]{1,4})\s*$)");
        std::smatch parts;
        int major = -1;
        int minor = -1;
        int patch = -1;
        if (!std::regex_match(version, parts, pattern)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Motor %u returned malformed firmware version '%s'; activation aborted",
                         motor_ids_[i], version.c_str());
            return false;
        }
        major = std::stoi(parts[1].str());
        minor = std::stoi(parts[2].str());
        patch = std::stoi(parts[3].str());
        const bool compatible = major > required_major ||
            (major == required_major && minor > required_minor) ||
            (major == required_major && minor == required_minor && patch >= required_patch);
        if (!compatible) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Motor %u firmware '%s' is below required 1.16.07; hardware remains inactive",
                         motor_ids_[i], version.c_str());
            return false;
        }
        RCLCPP_INFO(get_node()->get_logger(), "Motor %u firmware: %s",
                    motor_ids_[i], version.c_str());
    }
    return true;
}

int16_t JxHardware::angleToInt16(double angle) {
    // 将输入角度限制在[-180, 180]度范围内
    double clamped_angle = std::clamp(angle, ANGLE_MIN, ANGLE_MAX);    
    // 计算角度在范围内的比例（0~1）
    double ratio = (clamped_angle - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN);
    
    // === 新协议适配 START: CSP位置范围[-32668, 32668]对应[-180°, 180°] ===
    int32_t raw_value = RAW_MIN + ratio * (RAW_MAX - RAW_MIN);
    int32_t clamped_raw = std::clamp(raw_value, (int32_t)RAW_MIN, (int32_t)RAW_MAX);
    // === 新协议适配 END ===
    return static_cast<int16_t>(clamped_raw);
}

double JxHardware::int16ToAngle(int16_t int_val) {
    // 计算计数在协议范围内的比例（0~1）
    double ratio = (int_val - RAW_MIN) / static_cast<double>(RAW_MAX - RAW_MIN);
    
    // 将比例映射到角度范围[-180, 180]度
    double angle = ANGLE_MIN + ratio * (ANGLE_MAX - ANGLE_MIN);
    
    return std::round(angle * 1000) / 1000;
}

// === ROS2 Control 初始化接口 ===
hardware_interface::CallbackReturn JxHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
    const auto get_param = [&](const std::string& name, const std::string& default_val) {
        auto it = info_.hardware_parameters.find(name);
        return it != info_.hardware_parameters.end() ? it->second : default_val;
    };
    error_pub_ = get_node()->create_publisher<std_msgs::msg::Int64>("/Base_HardwareError", 10);
    RCLCPP_INFO(get_node()->get_logger(), "Error publisher created on /Base_HardwareError");
    // 解析核心参数
    can_interface_ = get_param("can_interface", "can0");
    check_firmware_version_ = get_param("check_firmware_version", "true") != "false";
    control_period_ms_ = std::stof(get_param("control_period_ms", "1")); // 默认1ms=1000Hz
    if (!std::isfinite(control_period_ms_) || control_period_ms_ < 0.1f || control_period_ms_ > 1000.f)
        throw std::invalid_argument("control_period_ms must be finite and in [0.1,1000]");

    max_position_step_nct_ = static_cast<int32_t>(
        std::stoi(get_param("max_position_step_nct", "90")));
    if (max_position_step_nct_ <= 0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'max_position_step_nct' must be > 0, got %d",
                     max_position_step_nct_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 加速度限幅：限制相邻两帧 delta 的变化量；0 关闭
    max_position_accel_nct_ = static_cast<int32_t>(
        std::stoi(get_param("max_position_accel_nct", "10")));
    if (max_position_accel_nct_ < 0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'max_position_accel_nct' must be >= 0, got %d",
                     max_position_accel_nct_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 解析电机ID列表
    std::string motor_ids_str = get_param("motor_ids", "");
    if (motor_ids_str.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'motor_ids' is required (e.g. '1,2' or '[1,2]')");
        return hardware_interface::CallbackReturn::ERROR;
    }
    motor_ids_str.erase(std::remove(motor_ids_str.begin(), motor_ids_str.end(), '['), motor_ids_str.end());
    motor_ids_str.erase(std::remove(motor_ids_str.begin(), motor_ids_str.end(), ']'), motor_ids_str.end());
    std::istringstream iss_ids(motor_ids_str);
    std::string token;
    motor_ids_.clear();
    while (std::getline(iss_ids, token, ',')) {
        const int id = std::stoi(token);
        if (id < 1 || id > 127 || std::find(motor_ids_.begin(), motor_ids_.end(), id) != motor_ids_.end())
            throw std::invalid_argument("motor_ids must be unique CANopen IDs in [1,127]");
        motor_ids_.push_back(static_cast<uint8_t>(id));
    }
    if (motor_ids_.empty() || motor_ids_.size() > MAX_MOTOR_COUNT ||
        motor_ids_.size() != info_.joints.size()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "motor_ids count (%zu) must match joints (%zu) and be in [1,%u]",
                     motor_ids_.size(), info_.joints.size(), MAX_MOTOR_COUNT);
        return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string default_mode = get_param("control_mode", "position");
    ControlMode configured_mode = ControlMode::CSP_POSITION;
    if (default_mode == "velocity") {
        configured_mode = ControlMode::CSV_VELOCITY;
    } else if (default_mode == "current") {
        configured_mode = ControlMode::CURRENT_TORQUE;
    } else if (default_mode != "position") {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'control_mode' must be 'position', 'velocity' or 'current', got '%s'",
                     default_mode.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    motor_control_modes_.assign(motor_ids_.size(), configured_mode);

    // 逗号/方括号列表 → motor id 列表（两个参数共用一套解析）。
    const auto parse_motor_id_list = [&](const std::string& param_name) {
        std::vector<uint8_t> parsed_ids;
        std::string raw = get_param(param_name, "");
        raw.erase(std::remove(raw.begin(), raw.end(), '['), raw.end());
        raw.erase(std::remove(raw.begin(), raw.end(), ']'), raw.end());
        std::istringstream stream(raw);
        std::string item;
        while (std::getline(stream, item, ',')) {
            if (item.find_first_not_of(" \t") == std::string::npos) {
                continue;
            }
            const int parsed_id = std::stoi(item);
            if (parsed_id < 1 || parsed_id > 127) {
                throw std::invalid_argument(param_name + " must be in [1,127]");
            }
            const uint8_t id = static_cast<uint8_t>(parsed_id);
            if (std::find(motor_ids_.begin(), motor_ids_.end(), id) == motor_ids_.end()) {
                throw std::invalid_argument(param_name + " contains motor " + std::to_string(parsed_id) +
                                            " not present in motor_ids");
            }
            parsed_ids.push_back(id);
        }
        return parsed_ids;
    };
    const std::vector<uint8_t> velocity_ids = parse_motor_id_list("velocity_motor_ids");
    const std::vector<uint8_t> current_ids = parse_motor_id_list("current_motor_ids");
    for (const auto id : velocity_ids) {
        if (std::find(current_ids.begin(), current_ids.end(), id) != current_ids.end()) {
            throw std::invalid_argument("motor " + std::to_string(id) +
                                        " is listed in both velocity_motor_ids and current_motor_ids");
        }
    }
    const auto apply_mode_override = [&](const std::vector<uint8_t>& ids, ControlMode mode) {
        for (const auto id : ids) {
            const auto it = std::find(motor_ids_.begin(), motor_ids_.end(), id);
            motor_control_modes_[static_cast<size_t>(std::distance(motor_ids_.begin(), it))] = mode;
        }
    };
    apply_mode_override(velocity_ids, ControlMode::CSV_VELOCITY);
    apply_mode_override(current_ids, ControlMode::CURRENT_TORQUE);
    waist_current_interlock_enabled_ = false;
    waist_current_fault_ = false;
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (isWaistMotor(i) && motor_control_modes_[i] == ControlMode::CURRENT_TORQUE)
            waist_current_interlock_enabled_ = true;
    }

    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        const auto declares = [&](const char* interface_name) {
            return std::any_of(info_.joints[i].command_interfaces.begin(),
                               info_.joints[i].command_interfaces.end(),
                               [interface_name](const auto& interface) {
                                   return interface.name == interface_name;
                               });
        };
        if (motor_control_modes_[i] == ControlMode::CSV_VELOCITY && !declares("velocity")) {
            throw std::invalid_argument("CSV joint must declare a velocity command interface");
        }
        // Current-loop impedance uses position and velocity as references, not effort commands.
        if (motor_control_modes_[i] == ControlMode::CURRENT_TORQUE) {
            if (!declares("position")) {
                throw std::invalid_argument(
                    "Current-loop joint must declare a position command interface (q_d reference)");
            }
            if (!declares("velocity")) {
                throw std::invalid_argument(
                    "Current-loop joint must declare a velocity command interface (q_dot_d reference)");
            }
        }
    }
    max_velocity_rpm_ = std::stoi(get_param("max_velocity_rpm", "32767"));
    if (max_velocity_rpm_ <= 0 || max_velocity_rpm_ > 32767) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'max_velocity_rpm' must be in [1,32767], got %d",
                     max_velocity_rpm_);
        return hardware_interface::CallbackReturn::ERROR;
    }
    // 电流环模式专用（mA）：模块自身不平滑也不限幅，所以这两项必须在驱动里做。
    max_current_ma_ = std::stoi(get_param("max_current_ma", "3000"));
    if (max_current_ma_ <= 0 || max_current_ma_ > 32767) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'max_current_ma' must be in [1,32767], got %d", max_current_ma_);
        return hardware_interface::CallbackReturn::ERROR;
    }
    current_slew_rate_ma_s_ = std::stod(get_param("current_slew_rate_ma_s", "0"));
    if (!std::isfinite(current_slew_rate_ma_s_) || current_slew_rate_ma_s_ < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'current_slew_rate_ma_s' must be finite and >= 0 (0 = disabled), got %g",
                     current_slew_rate_ma_s_);
        return hardware_interface::CallbackReturn::ERROR;
    }
    // Consecutive read() cycles without healthy feedback before latching a motor to CSP hold.
    feedback_timeout_cycles_ = std::stoi(get_param("feedback_timeout_cycles", "10"));
    if (feedback_timeout_cycles_ < 1 || feedback_timeout_cycles_ > 10000) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'feedback_timeout_cycles' must be in [1,10000], got %d",
                     feedback_timeout_cycles_);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ── 底层阻抗层参数（单位全部是 mA；1 个值 = 全部电机同值）──────────────────
    const auto parse_gain_list = [&](const char* param_name, std::vector<double>& out,
                                     bool allow_negative) {
        std::vector<double> parsed;
        std::string raw = get_param(param_name, "");
        raw.erase(std::remove(raw.begin(), raw.end(), '['), raw.end());
        raw.erase(std::remove(raw.begin(), raw.end(), ']'), raw.end());
        std::istringstream stream(raw);
        std::string item;
        while (std::getline(stream, item, ',')) {
            if (item.find_first_not_of(" \t") != std::string::npos) {
                parsed.push_back(std::stod(item));
            }
        }
        if (parsed.empty()) {
            parsed.assign(motor_ids_.size(), 0.0);
        } else if (parsed.size() == 1) {
            parsed.assign(motor_ids_.size(), parsed.front());
        } else if (parsed.size() != motor_ids_.size()) {
            throw std::invalid_argument(std::string(param_name) + " needs one value or " +
                                        std::to_string(motor_ids_.size()) + " values");
        }
        for (const double value : parsed) {
            if (!std::isfinite(value) || (!allow_negative && value < 0.0)) {
                throw std::invalid_argument(std::string(param_name) + " values must be finite" +
                                            (allow_negative ? "" : " and >= 0"));
            }
        }
        out = std::move(parsed);
    };
    parse_gain_list("impedance_kp_ma_rad", impedance_kp_ma_rad_, false);
    parse_gain_list("impedance_kd_ma_s_rad", impedance_kd_ma_s_rad_, false);
    parse_gain_list("friction_coulomb_ma", friction_coulomb_ma_, false);
    parse_gain_list("friction_viscous_ma_s_rad", friction_viscous_ma_s_rad_, false);
    parse_gain_list("friction_offset_ma", friction_offset_ma_, true);
    parse_gain_list("gravity_feedforward_ma", gravity_feedforward_ma_, true);
    friction_vel_eps_rad_s_ = std::stod(get_param("friction_vel_eps_rad_s", "0.002"));
    if (!std::isfinite(friction_vel_eps_rad_s_) || friction_vel_eps_rad_s_ < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'friction_vel_eps_rad_s' must be finite and >= 0, got %g",
                     friction_vel_eps_rad_s_);
        return hardware_interface::CallbackReturn::ERROR;
    }
    // 摩擦前馈取哪个速度 + 幅值上限（不配 = 老行为：实测速度、不限幅）。
    const std::string ff_velocity = get_param("friction_ff_velocity", "measured");
    if (ff_velocity != "measured" && ff_velocity != "reference") {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'friction_ff_velocity' must be 'measured' or 'reference', got '%s'",
                     ff_velocity.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    friction_ff_use_reference_vel_ = (ff_velocity == "reference");
    friction_ff_max_ma_ = std::stod(get_param("friction_ff_max_ma", "0"));
    if (!std::isfinite(friction_ff_max_ma_) || friction_ff_max_ma_ < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'friction_ff_max_ma' must be finite and >= 0 (0 = unlimited), got %g",
                     friction_ff_max_ma_);
        return hardware_interface::CallbackReturn::ERROR;
    }
    // 电流环必须加载有效的重力表；CSP/CSV 可不配置。
    // 它是**无控制器时唯一能把腰顶住**的东西：手臂出错 ⇒ WBC 激活不了 ⇒ 没人发位置参考、
    // 没人补重力 ⇒ 腰 1/2/3 只剩弹簧。
    gravity_ff_table_path_ = get_param("gravity_ff_table", "");
    if (!loadGravityFfTable(gravity_ff_table_path_)) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    // 上层推送的【精确】重力前馈（含真实手臂姿态）——见 jx_hardware.h。留空 = 关闭。
    gravity_ff_topic_ = get_param("gravity_ff_topic", "");
    {
        const std::string timeout = get_param("gravity_ff_topic_timeout_s", "0.5");
        gravity_ff_topic_timeout_s_ = std::stod(timeout);
        if (!std::isfinite(gravity_ff_topic_timeout_s_) || gravity_ff_topic_timeout_s_ <= 0.0) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Parameter 'gravity_ff_topic_timeout_s' must be finite and > 0, got '%s'",
                         timeout.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    gravity_ff_transition_s_ = std::stod(get_param("gravity_ff_transition_s", "1.0"));
    if (!std::isfinite(gravity_ff_transition_s_) || gravity_ff_transition_s_ < 0.0) {
        throw std::invalid_argument("gravity_ff_transition_s must be finite and >= 0");
    }
    if (!gravity_ff_topic_.empty()) {
        // ⚠️ 这是 controller_manager 的节点，回调跑在 CM 的 executor 上（不是 read/write
        //    那个更新线程）。缓存用 mutex 保护；阻抗层每周期只读一次，开销可忽略。
        gravity_ff_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
            gravity_ff_topic_, rclcpp::QoS(1).best_effort(),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (!msg) {
                    return;
                }
                // 话题布局：下标 = 电机 ID − 1（头部 5/6 也在里面，驱动只取自己的电机）。
                // ⚠️ 必须**全部覆盖 + 全部有限**，任一不满足就**整条丢弃**：
                //    短消息/NaN 会让某些轴的重力前馈变 0，而 NaN 又会被阻抗律末尾的
                //    `if (!std::isfinite(current)) current = 0.0;` 兜底清成 **0 mA** ——
                //    对重力关节就是自由下坠。丢弃后 stamp 不更新，超时就自然退回查表（保命层）。
                const size_t needed =
                    static_cast<size_t>(*std::max_element(motor_ids_.begin(), motor_ids_.end()));
                if (msg->data.size() < needed) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                                         "重力前馈话题长度 %zu < 需要的 %zu，整条丢弃（继续用查表）",
                                         msg->data.size(), needed);
                    return;
                }
                std::vector<double> by_slot(motor_ids_.size(), 0.0);
                for (size_t i = 0; i < motor_ids_.size(); ++i) {
                    const double v = msg->data[static_cast<size_t>(motor_ids_[i]) - 1];
                    if (!std::isfinite(v)) {
                        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                                             "重力前馈话题含非有限值（电机 %u = %g），整条丢弃（继续用查表）",
                                             motor_ids_[i], v);
                        return;
                    }
                    by_slot[i] = v;
                }
                std::lock_guard<std::mutex> lock(gravity_ff_mutex_);
                gravity_ff_topic_ma_ = std::move(by_slot);
                gravity_ff_topic_stamp_ = std::chrono::steady_clock::now();
            });
        // 动态分量独立接收/过期，不能沿用静态重力的 0.5 s 保持时间。
        dynamic_ff_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
            gravity_ff_topic_ + "/dynamic", rclcpp::QoS(1).best_effort(),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                const size_t needed =
                    static_cast<size_t>(*std::max_element(motor_ids_.begin(), motor_ids_.end()));
                if (!msg || msg->data.size() < needed) return;
                std::vector<double> by_slot(motor_ids_.size(), 0.0);
                for (size_t i = 0; i < motor_ids_.size(); ++i) {
                    const double value = msg->data[static_cast<size_t>(motor_ids_[i]) - 1];
                    if (!std::isfinite(value)) return;
                    by_slot[i] = value;
                }
                std::lock_guard<std::mutex> lock(gravity_ff_mutex_);
                dynamic_ff_topic_ma_ = std::move(by_slot);
                dynamic_ff_topic_stamp_ = std::chrono::steady_clock::now();
            });
        RCLCPP_INFO(get_node()->get_logger(),
                    "Waist gravity FF topic: %s (fresh <= %.2f s) —— 新鲜时优先于查表",
                    gravity_ff_topic_.c_str(), gravity_ff_topic_timeout_s_);
    }
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (motor_control_modes_[i] != ControlMode::CURRENT_TORQUE) {
            continue;
        }
        RCLCPP_INFO(get_node()->get_logger(),
                    "Motor %u impedance layer: Kp=%.1f mA/rad, Kd=%.1f mA*s/rad, "
                    "Coulomb=%.0f mA, viscous=%.0f mA*s/rad, offset=%.0f mA, gravity=%.0f mA, "
                    "v_eps=%.4f rad/s, limit=+/-%d mA, slew=%.0f mA/s, ff_vel=%s, ff_max=%.0f mA",
                    motor_ids_[i], impedance_kp_ma_rad_[i], impedance_kd_ma_s_rad_[i],
                    friction_coulomb_ma_[i], friction_viscous_ma_s_rad_[i],
                    friction_offset_ma_[i], gravity_feedforward_ma_[i],
                    friction_vel_eps_rad_s_, max_current_ma_, current_slew_rate_ma_s_,
                    friction_ff_use_reference_vel_ ? "reference" : "measured",
                    friction_ff_max_ma_);
    }

    gear_ratios_.assign(motor_ids_.size(), 1.0);
    std::string ratios = get_param("gear_ratios", "");
    ratios.erase(std::remove(ratios.begin(), ratios.end(), '['), ratios.end());
    ratios.erase(std::remove(ratios.begin(), ratios.end(), ']'), ratios.end());
    if (!ratios.empty()) {
        std::vector<double> parsed;
        std::istringstream iss_ratios(ratios);
        while (std::getline(iss_ratios, token, ',')) {
            if (token.find_first_not_of(" \t") != std::string::npos) {
                parsed.push_back(std::stod(token));
            }
        }
        if (parsed.size() == 1) {
            std::fill(gear_ratios_.begin(), gear_ratios_.end(), parsed.front());
        } else if (parsed.size() == motor_ids_.size()) {
            gear_ratios_ = std::move(parsed);
        } else {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "gear_ratios needs one value or %zu values, got %zu",
                         motor_ids_.size(), parsed.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    if (std::any_of(gear_ratios_.begin(), gear_ratios_.end(),
                    [](double ratio) { return !std::isfinite(ratio) || ratio <= 0.0; })) {
        RCLCPP_ERROR(get_node()->get_logger(), "All gear_ratios must be finite and > 0");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 初始化数据存储
    size_t motor_count = motor_ids_.size();
    joint_positions_.resize(motor_count, 0.0);
    joint_velocities_.resize(motor_count, 0.0);
    joint_efforts_.resize(motor_count, 0.0);
    joint_position_commands_.resize(motor_count, 0.0);
    joint_velocity_commands_.resize(motor_count, 0.0);
    position_reference_.assign(motor_count, 0.0);
    velocity_reference_.assign(motor_count, 0.0);
    latched_hold_positions_.assign(motor_count, 0);
    transmit_fault_ = false;
    transmit_retry_started_ = {};
    raw_position_commands_.resize(motor_count, 0);
    raw_velocity_commands_.resize(motor_count, 0);
    raw_current_commands_.resize(motor_count, 0);
    target_raw_position_commands_.resize(motor_count, 0);
    last_sent_raw_commands_.resize(motor_count, 0);
    last_sent_deltas_.resize(motor_count, 0);
    last_sent_current_commands_.resize(motor_count, 0.0);
    motor_unhealthy_count_.assign(motor_count, 0);
    motor_ready_.assign(motor_count, false);
    motor_hold_latched_.assign(motor_count, 0);   // 锁存只在这里（真正重新初始化）清

    running_ = false;
    is_first_command_ = true;

    // 打印初始化信息
    RCLCPP_INFO(get_node()->get_logger(), "JxHardware initialized:");
    RCLCPP_INFO(get_node()->get_logger(), "  CAN Interface: %s", can_interface_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Control Period: %.2f ms", control_period_ms_);
    RCLCPP_INFO(get_node()->get_logger(), "  Control Mode: %s", controlModeName());
    if (hasVelocityControl()) {
        RCLCPP_INFO(get_node()->get_logger(), "  Max velocity: %d motor RPM", max_velocity_rpm_);
    }
    RCLCPP_INFO(get_node()->get_logger(), "  Max position step: %d nct/cycle", max_position_step_nct_);
    RCLCPP_INFO(get_node()->get_logger(), "  Max position accel: %d nct/cycle^2%s",
                max_position_accel_nct_,
                max_position_accel_nct_ == 0 ? " (disabled)" : "");
    RCLCPP_INFO(get_node()->get_logger(), "  Motor IDs (count: %zu):", motor_ids_.size());

    for (uint8_t id : motor_ids_) {
        RCLCPP_INFO(get_node()->get_logger(), "    - %d", id);
    }
    if (hasVelocityControl() && ratios.empty()) {
        RCLCPP_WARN(get_node()->get_logger(),
                    "CSV is using the default 1:1 gear ratio; set 'gear_ratios' for accurate rad/s");
    }
    
    ///  配置can fd 
    std::string can_config_cmd = 
        "sudo ip link set " + can_interface_ + " down && "  
        "sudo ip link set " + can_interface_ + " up type can "
        "bitrate 1000000 "          // 仲裁段波特率1M
        "dbitrate 5000000 "         // 数据段波特率5M
        "fd on ";                    // 启用FD模式
    
    int config_ret = system(can_config_cmd.c_str());  
    if (config_ret == -1 || WEXITSTATUS(config_ret) != 0) {
        RCLCPP_ERROR(get_node()->get_logger(), " [CAN FD] Failed to configure %s!", can_interface_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 延时100ms，确保内核完成设备配置

    return hardware_interface::CallbackReturn::SUCCESS;
    } catch (const std::exception& error) {
        RCLCPP_ERROR(get_node()->get_logger(), "Invalid Juxie configuration: %s", error.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
}

// === 激活硬件（初始化CAN+使能电机+获取初始状态）===
// ── 腰部重力前馈表：加载 + 插值 ─────────────────────────────────────────────
// 为什么在驱动里算：控制器（ocs2_wbc_controller）必须同时 claim 20 个关节，手臂
// （M6_CCS）一出错它就激活不了；而驱动照样会把腰切进电流环 ⇒ 没人补重力 ⇒ 腰 1/2/3
// 只剩弹簧、下沉几度。只要模型在驱动里，**有没有控制器都不会沉**。
bool JxHardware::loadGravityFfTable(const std::string &path) {
    gravity_ff_table_n_ = 0;
    gravity_ff_table_data_.clear();
    gravity_ff_table_slots_.clear();
    if (path.empty()) {
        if (hasCurrentControl()) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Current-loop control requires gravity_ff_table; refusing initialization/activation");
            return false;
        }
        return true;
    }
    std::ifstream in(path);
    if (!in) {
        RCLCPP_ERROR(get_node()->get_logger(), "重力前馈表打不开: %s", path.c_str());
        return false;
    }
    // 表列顺序固定 = 电机 ID 1,2,3,4（= body_joint1~4），找出它们在本驱动里的下标
    for (const uint8_t id : {static_cast<uint8_t>(1), static_cast<uint8_t>(2),
                             static_cast<uint8_t>(3), static_cast<uint8_t>(4)}) {
        const auto it = std::find(motor_ids_.begin(), motor_ids_.end(), id);
        gravity_ff_table_slots_.push_back(
            it == motor_ids_.end() ? -1 : static_cast<int>(std::distance(motor_ids_.begin(), it)));
    }
    if (std::find(gravity_ff_table_slots_.begin(), gravity_ff_table_slots_.end(), -1) !=
        gravity_ff_table_slots_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "重力前馈表需要电机 ID 1,2,3,4 全部在本驱动的 motor_ids 里，实际个数=%zu",
                     motor_ids_.size());
        return false;
    }

    std::string line;
    // 只取非空、且不以 '#' 开头的行
    const auto next_data_line = [&](std::string &out) -> bool {
        while (std::getline(in, line)) {
            const auto pos = line.find_first_not_of(" \t\r");
            if (pos == std::string::npos || line[pos] == '#') {
                continue;
            }
            out = line.substr(pos);
            return true;
        }
        return false;
    };

    std::string first;
    if (!next_data_line(first)) {
        RCLCPP_ERROR(get_node()->get_logger(), "重力前馈表没有数据行: %s", path.c_str());
        return false;
    }
    int n1 = 0, n2 = 0, n3 = 0;
    {
        std::istringstream ss(first);
        if (!(ss >> n1 >> n2 >> n3) || n1 < 2 || n1 != n2 || n2 != n3) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "重力前馈表头必须是正方网格 'n n n'（且 n>=2），读到 '%s'", first.c_str());
            return false;
        }
    }
    const auto read3 = [&](double *dst) -> bool {
        std::string l;
        if (!next_data_line(l)) {
            return false;
        }
        std::istringstream ss(l);
        return static_cast<bool>(ss >> dst[0] >> dst[1] >> dst[2]);
    };
    if (!read3(gravity_ff_table_lo_) || !read3(gravity_ff_table_hi_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "重力前馈表缺 lo/hi 行: %s", path.c_str());
        return false;
    }

    const size_t n = static_cast<size_t>(n1);
    gravity_ff_table_data_.assign(n * n * n * 4, 0.0F);
    std::vector<char> seen(n * n * n, 0);
    size_t filled = 0;
    while (next_data_line(line)) {
        std::istringstream ss(line);
        int i1 = 0, i2 = 0, i3 = 0;
        double v0 = 0.0, v1 = 0.0, v2 = 0.0, v3 = 0.0;
        if (!(ss >> i1 >> i2 >> i3 >> v0 >> v1 >> v2 >> v3)) {
            continue;
        }
        if (i1 < 0 || i2 < 0 || i3 < 0 || i1 >= n1 || i2 >= n2 || i3 >= n3) {
            RCLCPP_ERROR(get_node()->get_logger(), "重力前馈表下标越界: '%s'", line.c_str());
            return false;
        }
        const size_t cell = (static_cast<size_t>(i1) * n + static_cast<size_t>(i2)) * n +
                            static_cast<size_t>(i3);
        // ⚠️ 重复点会「捷足先登」：重复点 + 缺失点让 filled 仍然等于 n³，但缺失点会
        //    留着 assign() 的 0 ⇒ 该点周围插值出一个错误的「无重力」值。
        if (seen[cell]) {
            RCLCPP_ERROR(get_node()->get_logger(), "重力前馈表有重复网格点 (%d,%d,%d): '%s'",
                         i1, i2, i3, line.c_str());
            return false;
        }
        // ⚠️ 必须校验有限性：`ss >> double` 会接受 nan/inf，而 NaN 一旦进到阻抗律会被
        //    `isfinite` 兜底清成 0 mA ⇒ 重力关节自由下坠。
        if (!std::isfinite(v0) || !std::isfinite(v1) || !std::isfinite(v2) ||
            !std::isfinite(v3)) {
            RCLCPP_ERROR(get_node()->get_logger(), "重力前馈表含非有限值: '%s'", line.c_str());
            return false;
        }
        seen[cell] = 1;
        const size_t base = cell * 4;
        gravity_ff_table_data_[base + 0] = static_cast<float>(v0);
        gravity_ff_table_data_[base + 1] = static_cast<float>(v1);
        gravity_ff_table_data_[base + 2] = static_cast<float>(v2);
        gravity_ff_table_data_[base + 3] = static_cast<float>(v3);
        ++filled;
    }
    if (filled != n * n * n) {
        RCLCPP_ERROR(get_node()->get_logger(), "重力前馈表不完整：期望 %zu 行，实际 %zu 行",
                     n * n * n, filled);
        return false;
    }
    gravity_ff_table_n_ = n1;
    RCLCPP_INFO(get_node()->get_logger(),
                "Waist gravity FF table loaded: %s  n=%d  q=(%zu,%zu,%zu)  "
                "lo=(%.3f, %.3f, %.3f) hi=(%.3f, %.3f, %.3f)  slots=[%d %d %d %d]",
                path.c_str(), n1, n * n * n, n, n, gravity_ff_table_lo_[0],
                gravity_ff_table_lo_[1], gravity_ff_table_lo_[2], gravity_ff_table_hi_[0],
                gravity_ff_table_hi_[1], gravity_ff_table_hi_[2], gravity_ff_table_slots_[0],
                gravity_ff_table_slots_[1], gravity_ff_table_slots_[2], gravity_ff_table_slots_[3]);
    return true;
}

bool JxHardware::gravityFeedforwardFromTable(std::vector<double> &out_ma) const {
    if (gravity_ff_table_n_ < 2 || gravity_ff_table_data_.empty() ||
        gravity_ff_table_slots_.size() != 4) {
        return false;
    }
    const int n = gravity_ff_table_n_;
    // 表只用腰 1/2/3 的角度（body_joint4 是 yaw，整圈对 τ_g1..3 的影响 ≤0.86°，已忽略）
    double q[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < 3; ++k) {
        const int slot = gravity_ff_table_slots_[static_cast<size_t>(k)];
        if (slot >= 0 && static_cast<size_t>(slot) < joint_positions_.size()) {
            q[k] = joint_positions_[static_cast<size_t>(slot)];
        }
    }
    int i0[3] = {0, 0, 0};
    double f[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < 3; ++k) {
        const double span = gravity_ff_table_hi_[k] - gravity_ff_table_lo_[k];
        double u = span != 0.0 ? (q[k] - gravity_ff_table_lo_[k]) / span * (n - 1) : 0.0;
        u = std::clamp(u, 0.0, static_cast<double>(n - 1));  // 超出表的行程就取边界
        const int iu = static_cast<int>(std::floor(u));
        i0[k] = std::min(iu, n - 2);
        f[k] = u - static_cast<double>(i0[k]);
    }
    out_ma.assign(motor_ids_.size(), 0.0);
    for (int da = 0; da <= 1; ++da) {
        for (int db = 0; db <= 1; ++db) {
            for (int dc = 0; dc <= 1; ++dc) {
                const double wgt = (da ? f[0] : 1.0 - f[0]) * (db ? f[1] : 1.0 - f[1]) *
                                   (dc ? f[2] : 1.0 - f[2]);
                if (wgt == 0.0) {
                    continue;
                }
                const size_t base =
                    ((static_cast<size_t>(i0[0] + da) * n + static_cast<size_t>(i0[1] + db)) * n +
                     static_cast<size_t>(i0[2] + dc)) * 4;
                for (size_t c = 0; c < 4; ++c) {
                    const int slot = gravity_ff_table_slots_[c];
                    if (slot >= 0) {
                        out_ma[static_cast<size_t>(slot)] +=
                            wgt * static_cast<double>(gravity_ff_table_data_[base + c]);
                    }
                }
            }
        }
    }
    return true;
}

bool JxHardware::gravityFeedforwardFromTopic(std::vector<double> &out_ma,
                                            std::vector<double>* dynamic_ma) const {
    std::lock_guard<std::mutex> lock(gravity_ff_mutex_);
    if (dynamic_ma != nullptr) dynamic_ma->clear();
    if (gravity_ff_topic_ma_.empty()) {
        return false;
    }
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                   gravity_ff_topic_stamp_)
                         .count();
    if (age < 0.0 || age > gravity_ff_topic_timeout_s_) {
        return false;  // 过期 ⇒ 退回查表（不叠加，避免重复补偿）
    }
    out_ma = gravity_ff_topic_ma_;
    const double dynamic_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - dynamic_ff_topic_stamp_).count();
    if (dynamic_ma != nullptr && dynamic_age >= 0.0 && dynamic_age <= dynamic_ff_timeout_s_) {
        *dynamic_ma = dynamic_ff_topic_ma_;
    }
    return true;
}

hardware_interface::CallbackReturn JxHardware::on_activate(
    const rclcpp_lifecycle::State & /* previous_state */) {
    RCLCPP_INFO(get_node()->get_logger(), "Activating JxHardware...");
    if (waist_current_fault_.load()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Waist group fault is latched; reinitialize hardware before reactivation");
        return hardware_interface::CallbackReturn::ERROR;
    }
    // Recheck before touching CAN: the table may have disappeared since on_init().
    if (hasCurrentControl() && !loadGravityFfTable(gravity_ff_table_path_)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Gravity table unavailable; current-loop hardware activation refused");
        return hardware_interface::CallbackReturn::ERROR;
    }
    gravity_ff_fault_ = false;
    locked_feedback_len_ = 0;
    is_first_command_ = true;
    // Reset health counters for this activation, but keep fault latches until reinitialization.
    motor_unhealthy_count_.assign(motor_ids_.size(), 0);
    motor_ready_.assign(motor_ids_.size(), false);

    // 初始化CAN FD接口（启用1M/5M速率）
    if (!initCanFd()) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN FD initialization failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (hasVelocityControl() && check_firmware_version_ && !checkFirmwareVersions()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Firmware compatibility check failed; deactivating JxHardware");
        ::close(can_socket_);
        can_socket_ = -1;
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 发送同步帧→获取初始状态（仅初始化时发送一次）
    sendSyncFrame();
    if (!readMotorStates(true)) { 
        RCLCPP_ERROR(get_node()->get_logger(), "Incomplete motor state initialization");
        ::close(can_socket_);
        can_socket_ = -1;
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 打印当前姿态下的重力前馈插值值（**不依赖任何控制器**），便于与离线
    // `scripts/identify_juxie_friction_gravity.py table` 打印的「核对用」数字逐位对照。
    if (gravity_ff_table_n_ >= 2) {
        std::vector<double> gtab;
        if (gravityFeedforwardFromTable(gtab)) {
            std::string msg;
            for (size_t i = 0; i < motor_ids_.size() && i < gtab.size(); ++i) {
                if (motor_control_modes_[i] != ControlMode::CURRENT_TORQUE) {
                    continue;
                }
                msg += (msg.empty() ? "" : ", ") + std::string("M") +
                       std::to_string(static_cast<int>(motor_ids_[i])) + "=" +
                       std::to_string(static_cast<long>(std::lround(gtab[i]))) + "mA";
            }
            RCLCPP_INFO(get_node()->get_logger(),
                        "Waist gravity FF at current pose (driver-side, no controller needed): [%s]",
                        msg.c_str());
        }
    }

    if (hasVelocityControl() && locked_feedback_len_ != 12) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "CSV velocity mode requires 12-byte feedback; detected %u bytes",
                     locked_feedback_len_);
        ::close(can_socket_);
        can_socket_ = -1;
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    transmit_fault_ = false;
    transmit_retry_started_ = {};
    // 初始化命令缓冲区（从当前反馈起步，避免首帧跃迁）
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        joint_position_commands_[i] = joint_positions_[i]; // 弧度
        resetPositionTarget(i);
        position_reference_[i] = joint_positions_[i];
        velocity_reference_[i] = 0.0;
        joint_velocity_commands_[i] = 0.0;
        raw_velocity_commands_[i] = 0;
    }
    // Initialize position references from measured positions and current slew limiting from measured Iq.
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        last_sent_current_commands_[i] = joint_efforts_[i];
    }
    gravity_ff_transition_.reset();
    {
        std::lock_guard<std::mutex> lock(gravity_ff_mutex_);
        dynamic_ff_topic_ma_.clear();
        dynamic_ff_topic_stamp_ = {};
    }
    // 启动控制线程（固定 control_period_ms_ 节奏发帧，与 CM 的 update_rate 无关）
    running_ = true;
    control_thread_ = std::thread(&JxHardware::controlLoop, this);

    RCLCPP_INFO(get_node()->get_logger(), "JxHardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// === 停用硬件（把电机切回 CSP 保持位置 + 关闭 CAN）===
hardware_interface::CallbackReturn JxHardware::on_deactivate(
    const rclcpp_lifecycle::State & /* previous_state */) {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating JxHardware...");

    running_ = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    // 切回 CSP 的帧是否至少提交成功。全部失败时返回 ERROR（不能宣称“已进入位置保持”）。
    bool csp_switch_send_failed = false;

    if (can_socket_ >= 0) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            std::fill(joint_velocity_commands_.begin(), joint_velocity_commands_.end(), 0.0);
            std::fill(raw_velocity_commands_.begin(), raw_velocity_commands_.end(), 0);
        }
        // Switch only non-CSP motors to position hold; stop CSP motors at their current position.
        std::vector<ControlMode> saved_modes;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            saved_modes = motor_control_modes_;
            for (size_t i = 0; i < motor_control_modes_.size(); ++i) {
                if (motor_control_modes_[i] != ControlMode::CSP_POSITION) {
                    motor_control_modes_[i] = ControlMode::CSP_POSITION;
                }
                resetPositionTarget(i);
            }
        }
        // Mode changes are asynchronous; send multiple frames and check that at least one was sent.
        // Protect shared mode and target state; sendMultiMotorCommand() acquires the mutex itself.
        int sent_ok = 0;
        for (int k = 0; k < 3; ++k) {
            if (sendMultiMotorCommand()) {
                ++sent_ok;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (sent_ok == 0) {
            csp_switch_send_failed = true;
            RCLCPP_ERROR(get_node()->get_logger(),
                         "切回 CSP 的 3 帧**全部提交失败**（CAN 写失败）⇒ 不保证电机已进入"
                         "位置保持。");
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            for (size_t i = 0; i < motor_control_modes_.size(); ++i) {
                // A failed shutdown send may have latched another motor.
                motor_control_modes_[i] = motor_hold_latched_[i]
                    ? ControlMode::CSP_POSITION : saved_modes[i];
            }
        }
        ::close(can_socket_);
        can_socket_ = -1;
    }

    if (csp_switch_send_failed) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "JxHardware deactivated（⚠️ 切回 CSP 的帧全部发送失败）");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(), "JxHardware deactivated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// === 导出接口 ===
std::vector<hardware_interface::StateInterface::ConstSharedPtr> 
JxHardware::on_export_state_interfaces() {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const std::string& joint_name = info_.joints[i].name;
        // 位置状态接口（弧度）
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
        // 速度状态接口（弧度/秒）
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));

    }
    RCLCPP_INFO(get_node()->get_logger(), "Exported %zu state interfaces (position+velocity)", state_interfaces.size());
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> 
JxHardware::on_export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const std::string& joint_name = info_.joints[i].name;
        // 保留 position 接口以维持整机控制器的关节索引；CSV 电机额外导出 velocity。
        command_interfaces.push_back(
            std::make_shared<hardware_interface::CommandInterface>(
                joint_name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
        if (motor_control_modes_[i] == ControlMode::CSV_VELOCITY) {
            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_commands_[i]));
        }
        // 电流环模式：额外导出 velocity 命令口 —— 上层写的就是普通速度命令 q̇_d，
        // 驱动内部不转发给电机速度环，而是和 position 口的 q_d 一起代进阻抗律算电流。
        // **不要导出 effort**：否则 CtrlInterfaces 的 MIX 自动检测会把整机切到 MIX 分支，
        // 反而改变了上层行为。
        if (motor_control_modes_[i] == ControlMode::CURRENT_TORQUE) {
            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_commands_[i]));
        }
    }
    RCLCPP_INFO(get_node()->get_logger(), "Exported %zu command interfaces (%s)",
                command_interfaces.size(), controlModeName());
    return command_interfaces;
}

// === ROS2 Control 读写接口 ===
hardware_interface::return_type JxHardware::read(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) {
    if (waist_current_fault_.load()) return hardware_interface::return_type::ERROR;
    if (gravity_ff_fault_.load()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Gravity table unavailable; requesting hardware error handling and controller deactivation");
        return hardware_interface::return_type::ERROR;
    }
    if (transmit_fault_.load()) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN transmit fault latched; requesting hardware error handling");
        return hardware_interface::return_type::ERROR;
    }
    if (!readMotorStates(false))
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (waist_current_interlock_enabled_)
                tripWaistCurrentInterlock("motor feedback/status failure");
        }
        RCLCPP_ERROR(get_node()->get_logger(), "Motor feedback/status failure; requesting hardware error handling");
        return hardware_interface::return_type::ERROR;
    }
    else
    {
        return hardware_interface::return_type::OK;
    }

}




hardware_interface::return_type JxHardware::write(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (transmit_fault_.load() || gravity_ff_fault_.load() || waist_current_fault_.load())
        return hardware_interface::return_type::ERROR;
    // Validate the complete snapshot before publishing any of it.
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (motor_hold_latched_[i]) continue;
        // position 命令（CSP / 电流环）与 velocity 命令（CSV / 电流环）都要是有限值。
        const bool needs_velocity = motor_control_modes_[i] == ControlMode::CSV_VELOCITY ||
                                    motor_control_modes_[i] == ControlMode::CURRENT_TORQUE;
        if (!std::isfinite(joint_position_commands_[i]) ||
            (needs_velocity && !std::isfinite(joint_velocity_commands_[i]))) {
            std::fill(raw_velocity_commands_.begin(), raw_velocity_commands_.end(), 0);
            // ⚠️ 刻意**不**清 raw_current_commands_：那是给电流环电机下发的力矩，清零 =
            //    重力关节自由下坠。这里只上报错误，由 CM 走 on_deactivate（它会先把电机
            //    切回 CSP 保持位置，是自包含的安全终态）。
            RCLCPP_ERROR(get_node()->get_logger(), "Non-finite command for motor %u",
                         motor_ids_[i]);
            return hardware_interface::return_type::ERROR;
        }
    }

    // 更新最新目标（CSP 的目标计数 / CSV 的电机 rpm）；实际下发由控制线程完成。
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        // 已锁存为 CSP 保持的电机：忽略上层命令，别把它再拉回 q_d（反馈不可信时那样做很危险）
        if (i < motor_hold_latched_.size() && motor_hold_latched_[i]) {
            continue;
        }
        if (motor_control_modes_[i] == ControlMode::CSP_POSITION) {
            const double angle_deg = joint_position_commands_[i] * 180.0 / M_PI;
            target_raw_position_commands_[i] = angleToInt16(angle_deg);
        } else if (motor_control_modes_[i] == ControlMode::CSV_VELOCITY) {
            const double motor_rpm = joint_velocity_commands_[i] * 60.0 /
                                     (2.0 * M_PI) * gear_ratios_[i];
            const long rounded = std::lround(std::clamp(
                motor_rpm, -static_cast<double>(max_velocity_rpm_),
                static_cast<double>(max_velocity_rpm_)));
            raw_velocity_commands_[i] = static_cast<int16_t>(rounded);
        } else {
            // 电流环：position/velocity 命令口是**参考**，真正的电流由控制线程的阻抗层算。
            // 这两个参考量只有电流环消费，CSP/CSV 轴不维护。
            position_reference_[i] = joint_position_commands_[i];
            velocity_reference_[i] = joint_velocity_commands_[i];
        }
    }

    return hardware_interface::return_type::OK;
}

// === CAN FD初始化（启用FD模式）===
bool JxHardware::initCanFd() {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN socket create failed: %s", strerror(errno));
        return false;
    }

    strcpy(ifr_.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr_) < 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN interface index failed: %s", strerror(errno));
        ::close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (::bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN bind failed: %s", strerror(errno));
        ::close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    // 启用CAN FD模式
    int enable_fd = 1;
    if (::setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN FD enable failed: %s", strerror(errno));
        ::close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    // 将 socket 设为非阻塞，避免 read() 在无数据时阻塞到超时，造成 CM 实时循环 Overrun
    int flags = ::fcntl(can_socket_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN set non-blocking failed: %s", strerror(errno));
        ::close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    RCLCPP_INFO(get_node()->get_logger(), "CAN FD initialized: %s (1M/5M, non-blocking)", can_interface_.c_str());
    return true;
}

// === 发送同步帧 ===
bool JxHardware::sendSyncFrame() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame)); 

    // 同步帧配置
    frame.can_id = static_cast<canid_t>(SYNC_FRAME_ID); 
    frame.len = 1; // 数据长度1字节
    frame.data[0] = 0x00; // 同步帧数据

    // 发送同步帧
    if (::write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Sync frame send failed: %s", strerror(errno));
        return false;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Sync frame sent (ID: 0x%X)", SYNC_FRAME_ID);
    return true;
}

// === 发送多控帧（协议0x200，控制多个电机）===
bool JxHardware::sendMultiMotorCommand() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 限速 + 限加速追赶最新目标：
    // 1) 速度：每周期 |delta| <= max_position_step_nct_
    // 2) 加速度：|delta - last_delta| <= max_position_accel_nct_（为 0 则跳过）
    // 3) 不越过当前目标（remaining 夹紧），避免指令超调
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (motor_control_modes_[i] != ControlMode::CSP_POSITION) {
            continue;
        }
        const int32_t target = target_raw_position_commands_[i];
        const int32_t last = last_sent_raw_commands_[i];
        const int32_t remaining = target - last;

        int32_t desired_delta = remaining;
        if (desired_delta > max_position_step_nct_) {
            desired_delta = max_position_step_nct_;
        } else if (desired_delta < -max_position_step_nct_) {
            desired_delta = -max_position_step_nct_;
        }

        int32_t delta = desired_delta;
        if (max_position_accel_nct_ > 0) {
            const int32_t last_delta = last_sent_deltas_[i];
            const int32_t min_delta = last_delta - max_position_accel_nct_;
            const int32_t max_delta = last_delta + max_position_accel_nct_;
            delta = std::clamp(desired_delta, min_delta, max_delta);
            delta = std::clamp(delta, -max_position_step_nct_, max_position_step_nct_);
        }

        // 本帧不要越过目标；若因此导致 delta 突变，以不超调优先
        if ((remaining >= 0 && delta > remaining) || (remaining <= 0 && delta < remaining)) {
            delta = remaining;
        }

        const int32_t sent = std::clamp(last + delta, static_cast<int32_t>(RAW_MIN),
                                        static_cast<int32_t>(RAW_MAX));
        raw_position_commands_[i] = static_cast<int16_t>(sent);
        last_sent_deltas_[i] = sent - last;
        last_sent_raw_commands_[i] = raw_position_commands_[i];
    }

    // ── 底层阻抗层：把上层的（位置, 速度）参考换成电流（mA）────────────────────
    //   I = Kp·(q_d − q) + Kd·(q̇_d − q̇) + g_ff + f_off + FF
    // 上层（OCS2/WBC）照旧只发位置/速度，换算在这里（每个 write() 一次 = CM update_rate）。
    //   FF = 0                                         当 |v_ff| < v_eps
    //      = sign(v_ff)·min(f_c + f_v·|v_ff|, ff_max)   否则
    //   v_ff：默认取**实测速度**；friction_ff_velocity=reference 时，|q̇_d| >= v_eps
    //         就改用**参考速度**（换向时抢先一步，真机跟踪误差实测好 2.4×），
    //         否则仍退回实测速度（被外力推动时前馈不会归零）。
    // 速度死区避免静止时的 1 rpm 量化噪声造成“正反馈自激”式抖动。
    // 腰部重力前馈，两层（**取其一，不叠加**）：
    //   ① 控制器活着 → 话题值（含真实手臂姿态，精确）
    //   ② 否则       → 驱动自己的查表（标称手臂姿态；不依赖任何控制器，保命）
    std::vector<double> gravity_topic_ma;
    std::vector<double> dynamic_topic_ma;
    const bool has_gravity_topic = gravityFeedforwardFromTopic(gravity_topic_ma, &dynamic_topic_ma);
    std::vector<double> gravity_table_ma;
    const bool has_gravity_table = gravityFeedforwardFromTable(gravity_table_ma);
    if (hasCurrentControl() && !has_gravity_table) {
        if (!gravity_ff_fault_.exchange(true)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "No valid gravity table for current-loop control; refusing current commands");
        }
        return false;
    }
    const double now_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    // Without a table there is no baseline to blend from; retain immediate topic use.
    const double topic_weight = gravity_ff_transition_.topicWeight(
        has_gravity_topic, now_s, has_gravity_table ? gravity_ff_transition_s_ : 0.0);
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (motor_control_modes_[i] != ControlMode::CURRENT_TORQUE) {
            continue;
        }
        // Sender only reads the snapshot committed by write().
        const double q = joint_positions_[i];
        const double q_d = position_reference_[i];
        const double v = joint_velocities_[i];
        const double v_d = velocity_reference_[i];
        const double table_i = has_gravity_table ? gravity_table_ma[i] : 0.0;
        const double topic_i = has_gravity_topic ? gravity_topic_ma[i] : table_i;
        const double gravity_auto_i = (1.0 - topic_weight) * table_i + topic_weight * topic_i;
        const double dynamic_i = i < dynamic_topic_ma.size() ? topic_weight * dynamic_topic_ma[i] : 0.0;
        double current = impedance_kp_ma_rad_[i] * (q_d - q) +
                         impedance_kd_ma_s_rad_[i] * (v_d - v) +
                         gravity_feedforward_ma_[i] + gravity_auto_i + dynamic_i + friction_offset_ma_[i];
        const double v_ff = (friction_ff_use_reference_vel_ &&
                             std::abs(v_d) >= friction_vel_eps_rad_s_) ? v_d : v;
        if (std::abs(v_ff) >= friction_vel_eps_rad_s_) {
            const double sign_v = v_ff > 0.0 ? 1.0 : -1.0;
            double friction_ff = friction_coulomb_ma_[i] +
                                 friction_viscous_ma_s_rad_[i] * std::abs(v_ff);
            if (friction_ff_max_ma_ > 0.0) {
                friction_ff = std::min(friction_ff, friction_ff_max_ma_);
            }
            current += sign_v * friction_ff;
        }
        if (!std::isfinite(current)) {
            current = 0.0;
        }
        raw_current_commands_[i] = static_cast<int16_t>(std::clamp(
            std::lround(current), -static_cast<long>(max_current_ma_),
            static_cast<long>(max_current_ma_)));
    }

    uint8_t data[CANFD_MAX_LEN] = {0};
    // 电流环：变化率限制（模块自身不平滑电流），按控制线程的固定周期折算每帧允许的增量。
    const double dt_s = static_cast<double>(control_period_ms_) * 1e-3;
    // ⚠️ 重新激活时不需要特殊处理：on_activate() 已经把 last_sent_current_commands_
    //    初始化成**实测 Iq 反馈**，所以这里永远从“电机真的在输出多少”开始爬。
    if (current_slew_rate_ma_s_ > 0.0) {
        const double max_step_ma = current_slew_rate_ma_s_ * dt_s;
        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            if (motor_control_modes_[i] != ControlMode::CURRENT_TORQUE) {
                continue;
            }
            const double target = static_cast<double>(raw_current_commands_[i]);
            const double last = last_sent_current_commands_[i];
            const double limited = last + std::clamp(target - last, -max_step_ma, max_step_ma);
            last_sent_current_commands_[i] = limited;
            raw_current_commands_[i] = static_cast<int16_t>(std::clamp(
                std::lround(limited), -static_cast<long>(max_current_ma_),
                static_cast<long>(max_current_ma_)));
        }
    } else {
        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            if (motor_control_modes_[i] == ControlMode::CURRENT_TORQUE) {
                last_sent_current_commands_[i] = static_cast<double>(raw_current_commands_[i]);
            }
        }
    }
    // 填充每个电机的控制包（7字节/个，共8个）
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        const size_t pkg_offset = i * 7;
        const ControlMode mode = motor_control_modes_[i];
        uint8_t control_byte;
        int16_t raw_command;
        if (locked_feedback_len_ == 7) {
            // 7 字节模块只认这一套控制字，模式由通道偏置决定。
            control_byte = is_first_command_ ? 0xD8 : 0xD0;
            raw_command = mode == ControlMode::CSP_POSITION ? raw_position_commands_[i]
                        : mode == ControlMode::CSV_VELOCITY ? raw_velocity_commands_[i]
                        : raw_current_commands_[i];
        } else {
            switch (mode) {
                case ControlMode::CSP_POSITION:
                    control_byte = is_first_command_ ? CTRL_WORD_CSP_CLEAR : CTRL_WORD_CSP;
                    raw_command = raw_position_commands_[i];
                    break;
                case ControlMode::CSV_VELOCITY:
                    control_byte = is_first_command_ ? CTRL_WORD_CSV_CLEAR : CTRL_WORD_CSV;
                    raw_command = raw_velocity_commands_[i];
                    break;
                default:
                    control_byte = is_first_command_ ? CTRL_WORD_CURRENT_CLEAR : CTRL_WORD_CURRENT;
                    raw_command = raw_current_commands_[i];
                    break;
            }
        }
        const uint16_t encoded = static_cast<uint16_t>(raw_command);
        data[pkg_offset + 0] = control_byte;
        data[pkg_offset + 1] = (encoded >> 8) & 0xFF;
        data[pkg_offset + 2] = encoded & 0xFF;
        data[pkg_offset + 3] = 0x00; // 目标参数2（位置/速度模式下为轮廓加减速，本驱动不用）
        data[pkg_offset + 4] = 0x00;
        data[pkg_offset + 5] = 0x00; // 目标前馈参数（电流环模式下为任意值）
        data[pkg_offset + 6] = 0x00;
        // === 新协议适配 END ===
    }

    // 填充56-63字节：电机ID
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        data[56 + i] = motor_ids_[i];
    }

    // 构造并发送CAN FD帧
    struct canfd_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = static_cast<canid_t>(MULTI_CTRL_ID);
    frame.len = CANFD_MAX_LEN;
    memcpy(frame.data, data, CANFD_MAX_LEN);
    const auto bytes = ::write(can_socket_, &frame, sizeof(frame));
    if (bytes != sizeof(frame)) {
        const int send_errno = bytes < 0 ? errno : EIO;
        const bool retryable = send_errno == EAGAIN || send_errno == EWOULDBLOCK ||
                               send_errno == ENOBUFS || send_errno == EINTR;
        const auto now = std::chrono::steady_clock::now();
        // Retry transient congestion at the normal send cadence, without busy looping.
        // Bound a continuous failure episode so a broken bus cannot be hidden forever.
        constexpr auto retry_window = std::chrono::milliseconds(100);
        if (retryable && !transmit_fault_.load()) {
            if (transmit_retry_started_.time_since_epoch().count() == 0)
                transmit_retry_started_ = now;
            if (now - transmit_retry_started_ < retry_window) {
                RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                    "Transient CAN send failure (%s); retrying, command delivery unconfirmed",
                    strerror(send_errno));
                return false;
            }
        }
        transmit_fault_.store(true);
        // Local desired state only: a failed bus cannot deliver this CSP request.
        for (size_t i = 0; i < motor_ids_.size(); ++i)
            latchMotorToCspHold(i, "CAN transmit fault");
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "CAN transmit fault (%s); CSP hold requested locally, delivery/mode unconfirmed; "
            "motor may still execute the last current command", strerror(send_errno));
        return false;
    }
    transmit_retry_started_ = {};

    // 第一次发送后更新标志
    if (is_first_command_) {
        RCLCPP_INFO(get_node()->get_logger(), "Sent first %s command", controlModeName());
        is_first_command_ = false;
    }

    return true;
}

// === 读取电机状态（非阻塞读取+缓存所有反馈）+ 错误处理 ===
bool JxHardware::readMotorStates(bool strict_check) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 创建临时映射表存储最新反馈
    std::unordered_map<uint8_t, canfd_frame> feedback_map;

    struct canfd_frame frame;

    // strict_check（仅 on_activate 首次读）：同步帧发出后反馈可能尚未到达，非阻塞 read 会立刻空返回。
    // 在此模式下轮询等待收齐所有电机反馈；正常运行 read(false) 仍只 drain 一次，不拖 CM 周期。
    const auto strict_deadline =
        strict_check ? (std::chrono::steady_clock::now() + std::chrono::milliseconds(200))
                     : std::chrono::steady_clock::now();

    for (;;) {
        // 读取当前 RX 队列中所有可用帧（非阻塞，无数据即退出内层循环）
        while (true) {
            ssize_t nbytes = ::read(can_socket_, &frame, sizeof(frame));
            if (nbytes <= 0) {
                if (nbytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                          1000, "CAN read error: %s", strerror(errno));
                }
                break;
            }

            // 检查是否是目标电机的反馈帧 (ID: 0x300+执行器ID)
            if (frame.can_id >= SINGLE_FEEDBACK_BASE && frame.can_id < SINGLE_FEEDBACK_BASE + 256) {
                uint8_t motor_id = frame.can_id - SINGLE_FEEDBACK_BASE;
                if (std::find(motor_ids_.begin(), motor_ids_.end(), motor_id) != motor_ids_.end()) {
                    feedback_map[motor_id] = frame;
                }
            } else if (frame.can_id >= 0x80 && frame.can_id < 0x80 + 256) {
                uint8_t motor_id = frame.can_id - 0x80;
                if (std::find(motor_ids_.begin(), motor_ids_.end(), motor_id) != motor_ids_.end()) {
                    parseAndReportEmergencyFrame(motor_id, frame);
                    running_ = false;
                    return false;
                }
            }
        }

        if (!strict_check) {
            break;
        }
        if (feedback_map.size() >= motor_ids_.size()) {
            break;
        }
        if (std::chrono::steady_clock::now() >= strict_deadline) {
            break;
        }
        struct pollfd pfd = {};
        pfd.fd = can_socket_;
        pfd.events = POLLIN;
        (void)::poll(&pfd, 1, 2);
    }

    if(feedback_map.size() != motor_ids_.size() && strict_check) {
        if (error_pub_) {
            std_msgs::msg::Int64 msg;
            msg.data = static_cast<int64_t>(0x300200);
            error_pub_->publish(msg);
            RCLCPP_INFO(get_node()->get_logger(), "Published hardware error: 0x%06X", 0x300200);
        }

        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                            1000, "feedback_map size is %zu, motor ids size is: %zu", 
                            feedback_map.size(), motor_ids_.size());
        return false;
    }

    if (strict_check && locked_feedback_len_ == 0 && !feedback_map.empty()) {
        const uint8_t first_len = feedback_map.begin()->second.len;
        for (const auto& item : feedback_map) {
            if (item.second.len != first_len) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Strict check failed: inconsistent feedback frame lengths (%u vs %u)",
                             first_len, item.second.len);
                running_ = false;
                return false;
            }
        }
        if (first_len != 7 && first_len != 12) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Strict check failed: unsupported feedback frame length %u (expected 7 or 12)",
                         first_len);
            running_ = false;
            return false;
        }
        locked_feedback_len_ = first_len;
        RCLCPP_INFO(get_node()->get_logger(),
                    "Locked feedback frame length to %u bytes", locked_feedback_len_);
        if (locked_feedback_len_ == 7 && hasCurrentControl()) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Current-loop mode requires the 12-byte feedback protocol (mode echo + Iq in mA), "
                         "but this module answers with 7-byte frames");
            running_ = false;
            return false;
        }
    }

    // 12/7 字节协议共用的健康度与联锁判定。返回 false = 必须停发整帧（腰部报错）。
    const auto applyHealth = [&](size_t index, uint8_t motor_id, bool ready, bool enabled, bool error) {
        if (isWaistMotor(index)) {
            motor_ready_[index] = ready;   // 只有腰部联锁消费这份状态
        }
        if (!enabled) {
            // 「未使能」对承重关节等于已经在掉；计入健康度，连续超时后锁存成 CSP 保持。
            ++motor_unhealthy_count_[index];
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                5000, "Motor %d is not enabled", motor_id);
        }
        else {
            // 只有「使能」的帧才清零健康计数（否则持续异常会累不上去）
            motor_unhealthy_count_[index] = 0;
        }
        if (!error) {
            return true;
        }
        // 只有腰部承重：腰部报错停发整帧（整组由联锁处理）；头部只锁存该轴。
        if (isWaistMotor(index)) {
            running_ = false;
            return false;
        }
        latchMotorToCspHold(index, "motor error");
        return true;
    };

    // 处理缓存的反馈数据
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        uint8_t id = motor_ids_[i];
        auto it = feedback_map.find(id);
        
        if (it != feedback_map.end()) {
            // ⚠️ 健康计数**不在这里**清零：收到帧 ≠ 帧健康。下面 enable/error 检查之后才
            //    决定清不清（否则「持续未使能」的计数器每拍被清一次，永远到不了阈值）。
            const canfd_frame& frame = it->second;
            if (locked_feedback_len_ > 0 && frame.len != locked_feedback_len_) {
                RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                   1000, "Feedback for motor %d frame length changed: %d (expected %u)",
                                   id, frame.len, locked_feedback_len_);
                running_ = false;
                return false;
            }

            const uint8_t protocol_len = (locked_feedback_len_ > 0) ? locked_feedback_len_ : frame.len;
            if (protocol_len == 12) {
                if (frame.len < 12) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                       1000, "Feedback for motor %d has insufficient data length: %d (<12)",
                                       id, frame.len);
                    ++motor_unhealthy_count_[i];   // 帧不可解析 = 不健康
                    continue;
                }
                int16_t raw_pos = (frame.data[0] << 8) | frame.data[1];
                double angle_deg = int16ToAngle(raw_pos);
                joint_positions_[i] = angle_deg * M_PI / 180.0;

                int16_t raw_vel = (frame.data[2] << 8) | frame.data[3];
                joint_velocities_[i] = raw_vel * 2 * M_PI / 60.0 / gear_ratios_[i];

                int16_t raw_current = (frame.data[4] << 8) | frame.data[5];
                joint_efforts_[i] = static_cast<double>(raw_current); // 新协议反馈Iq电流，单位mA

                uint16_t error_code = frame.data[6] | (static_cast<uint16_t>(frame.data[7]) << 8);
                const uint8_t status_byte = frame.data[11];
                const bool enable = (status_byte & 0x80) != 0;
                const bool error = (status_byte & 0x20) != 0 || error_code != 0;
                const bool brake_released = (status_byte & 0x40) != 0;
                if (error) {
                    if (error_code != 0) {
                        parseAndReportErrorCode(id, error_code, "feedback frame");
                    }
                    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                        1000, "Motor %d has error, error code: 0x%04X, status: 0x%02X",
                                        id, error_code, status_byte);
                }
                if (!applyHealth(i, id, enable && brake_released && !error, enable, error)) {
                    return false;
                }
            } else if (protocol_len == 7) {
                if (frame.len < 7) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                       1000, "Feedback for motor %d has insufficient data length: %d (<7)",
                                       id, frame.len);
                    ++motor_unhealthy_count_[i];   // 帧不可解析 = 不健康
                    continue;
                }
                int16_t raw_pos = (frame.data[0] << 8) | frame.data[1];
                double angle_deg = int16ToAngle(raw_pos);
                joint_positions_[i] = angle_deg * M_PI / 180.0;

                int16_t raw_vel = (frame.data[2] << 8) | frame.data[3];
                joint_velocities_[i] = raw_vel * 2 * M_PI / 60.0 / gear_ratios_[i];

                joint_efforts_[i] = 0.0;
                const uint8_t status_byte = frame.data[6];
                const bool enable = (status_byte & 0x20) != 0;
                const bool error = (status_byte & 0x08) != 0;
                if (error) {
                    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                        1000, "Motor %d has error, error code: 0x%02X", id, status_byte);
                }
                if (!applyHealth(i, id, enable && !error, enable, error)) {
                    return false;
                }
            } else {
                RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                    1000, "Unsupported feedback protocol length %u for motor %d",
                                    protocol_len, id);
                running_ = false;
                return false;
            }
        } else {
            // Retain the last command while counting missed feedback; do not send zero current.
            ++motor_unhealthy_count_[i];
            if (motor_unhealthy_count_[i] == 1) {
                // 单拍缺失是**正常**的（模块是异步回帧，回复可能落在本轮排空之后；
                // 实测 500 Hz 下约 0.2% 的拍会缺，且越靠后的电机 ID 越容易缺）⇒ 只打
                // DEBUG。只有在连续缺失到 feedback_timeout_cycles_ 时才会 WARN/ERROR。
                RCLCPP_DEBUG(get_node()->get_logger(), "Motor %u 本拍无反馈", id);
            }
        }
    }

    if (waist_current_interlock_enabled_) {
        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            if (!isWaistMotor(i)) continue;
            // 只在这里判“本拍就已确定不可用”的条件。反馈丢失由下面的连续超时判定，
            // 但一旦锁存就经 latchMotorToCspHold() 升级为整组：腰部电流按完整 4 轴算出，
            // 少一轴出力时其余轴的前馈/阻抗不再成立。
            if (!motor_ready_[i] || motor_hold_latched_[i]) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Waist motor %u not ready (enabled/brake/feedback); group current control refused",
                             motor_ids_[i]);
                // During startup no current command has been sent. Runtime failures latch the group.
                if (!strict_check) tripWaistCurrentInterlock("waist motor not ready");
                return false;
            }
        }
    }

    // ── 反馈健康度 → 锁存成「CSP 保持最后一帧位置」─────────────────────
    // 不做自动恢复：一旦不信任这条通道，就让模块自己抱住，直到重启驱动。
    size_t hold_count = 0;
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (motor_hold_latched_[i]) {
            ++hold_count;
            continue;
        }
        if (motor_unhealthy_count_[i] >= feedback_timeout_cycles_) {
            latchMotorToCspHold(i, "feedback timeout");
            ++hold_count;
        }
    }
    if (!motor_ids_.empty() && hold_count == motor_ids_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "全部 %zu 个电机反馈均不可用 → 上报硬件错误（控制器将被停掉；"
                     "各轴已本地锁存 CSP 保持请求，未确认电机执行）",
                     motor_ids_.size());
        return false;
    }
    // 腰部整组可能是在本次调用里被单轴锁存升级触发的；本拍就上报，不等下一拍。
    if (waist_current_fault_.load()) {
        return false;
    }

    return true;
}

bool JxHardware::isWaistMotor(size_t index) const {
    return index < motor_ids_.size() && motor_ids_[index] >= 1 && motor_ids_[index] <= 4;
}

void JxHardware::tripWaistCurrentInterlock(const char* reason) {
    waist_current_fault_.store(true);
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (isWaistMotor(i))
            latchOneMotorToCspHold(i, reason);
    }
}

void JxHardware::latchMotorToCspHold(size_t index, const char* reason) {
    // ⚠️ 调用方必须已持有 state_mutex_。
    // 腰部电流环按“完整 4 轴”计算重力/惯量补偿：任何一轴退出电流环后，其余轴继续按
    // 该模型出力就是欠补偿，会直接倒下去。所以腰部单轴锁存必须升级成整组锁存。
    if (waist_current_interlock_enabled_ && isWaistMotor(index)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Motor %u: %s; waist group shares one gravity model, latching all waist motors",
                     motor_ids_[index], reason);
        tripWaistCurrentInterlock(reason);
        return;
    }
    latchOneMotorToCspHold(index, reason);
}

void JxHardware::latchOneMotorToCspHold(size_t index, const char* reason) {
    // ⚠️ 调用方必须已持有 state_mutex_（readMotorStates() 内部调用）。
    if (index >= motor_hold_latched_.size() || motor_hold_latched_[index]) {
        return;
    }
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Motor %u: %s; locally latched CSP hold request @ %.4f rad "
                 "(command delivery and motor mode not confirmed)",
                 motor_ids_[index], reason, joint_positions_[index]);
    motor_hold_latched_[index] = 1;
    // 锁存 = 改用模块自己的 CSP 位置环抱住当前位置（所以模式就是 CSP）。
    // ⚠️ 这个改写**不恢复**：锁存只在 on_init 清，而 on_activate **不清**它 —— “这条通道
    //    不可信”不会因为一次重新激活就变可信（清掉只会让该轴静默变成 CSP 轴）。
    motor_control_modes_[index] = ControlMode::CSP_POSITION;
    // 目标 = 最后已知位置，且 last_sent 与之一致 ⇒ CSP 追赶逻辑的 remaining 为 0，
    // 它会原地不动地抱住这个位置。
    latched_hold_positions_[index] = angleToInt16(joint_positions_[index] * 180.0 / M_PI);
    resetPositionTarget(index);
    // Keep the prior current history; latching does not make physical current zero.
}

// Caller holds state_mutex_, or owns the inactive hardware during activation.
void JxHardware::resetPositionTarget(size_t index) {
    const int16_t target = motor_hold_latched_[index] ? latched_hold_positions_[index]
        : angleToInt16(joint_positions_[index] * 180.0 / M_PI);
    raw_position_commands_[index] = target;
    target_raw_position_commands_[index] = target;
    last_sent_raw_commands_[index] = target;
    last_sent_deltas_[index] = 0;
}

// === 新协议适配 START: 解析并发布新版16bit错误码 ===
void JxHardware::parseAndReportErrorCode(uint8_t motor_id, uint16_t error_code_raw, const std::string& source) {
    std::vector<std::string> error_messages;
    bool any_error = false;

    struct ErrorBitMap {
        uint16_t bit;        // 新协议故障位
        uint16_t fault_code; // 故障码低16位

        const char* message;
    };

    std::vector<ErrorBitMap> error_map = {
        {0x0001, 0x0001, "过压"},
        {0x0002, 0x0002, "欠压"},
        {0x0004, 0x0003, "过温"},
        {0x0008, 0x0004, "堵转"},
        {0x0010, 0x0005, "过载"},
        {0x0020, 0x0006, "速度跟踪误差过大"},
        {0x0040, 0x0007, "正限位保护"},
        {0x0080, 0x0008, "负限位保护"},
        {0x0100, 0x0009, "输出端编码器异常"},
        {0x0200, 0x000A, "位置跃迁过大"},
        {0x0400, 0x000B, "电流采样错误"},
        {0x0800, 0x000C, "电流跟踪误差超差"},
        {0x1000, 0x000D, "位置跟踪误差过大"},
    };

    // 遍历映射表，检测每个错误位
    for (const auto& mapping : error_map) {
        if (error_code_raw & mapping.bit) {
            any_error = true;

            // 组装错误码：0x3 << 24 | (motor_id << 16) | fault_code
            uint32_t error_code = (0x3 << 24) | (motor_id << 16) | mapping.fault_code;

            // 发布错误
            if (error_pub_) {
                std_msgs::msg::Int64 msg;
                msg.data = static_cast<int64_t>(error_code);
                error_pub_->publish(msg);
                RCLCPP_INFO(get_node()->get_logger(), "Published hardware error: 0x%06X", error_code);
            }

            error_messages.push_back(mapping.message);
        }
    }

    // 如果检测到紧急帧但没有任何已知错误位，发布未知错误（0x300100）
    if (!any_error) {
        uint32_t error_code = (0x3 << 24) | (motor_id << 16) | 0x0100; // 未知错误
        if (error_pub_) {
            std_msgs::msg::Int64 msg;
            msg.data = static_cast<int64_t>(error_code);
            error_pub_->publish(msg);
            RCLCPP_WARN(get_node()->get_logger(), "Published unknown hardware error: 0x%06X", error_code);
        }
        error_messages.push_back("未知错误");
    }

    if (!error_messages.empty()) {
        std::string error_str = "Motor " + std::to_string(motor_id) + " " + source + " errors: ";
        for (size_t i = 0; i < error_messages.size(); ++i) {
            error_str += error_messages[i];
            if (i < error_messages.size() - 1) {
                error_str += ", ";
            }
        }
        RCLCPP_ERROR(get_node()->get_logger(), "%s (raw: 0x%04X, SEVERE)", error_str.c_str(), error_code_raw);
    }
}
// === 新协议适配 END ===

// === 解析并报告紧急错误帧 ===
void JxHardware::parseAndReportEmergencyFrame(uint8_t motor_id, const canfd_frame& frame) {
    // 确保数据长度足够
    if (frame.len < 2) {
        RCLCPP_WARN(get_node()->get_logger(),
                   "Emergency frame for motor %d has insufficient data length: %d",
                   motor_id, frame.len);
        return;
    }

    if (locked_feedback_len_ == 7) {
        uint8_t error_byte0 = frame.data[0];
        uint8_t error_byte1 = frame.data[1];

        std::vector<std::string> error_messages;
        bool any_error = false;

        struct ErrorBitMap {
            uint8_t byte;
            uint8_t bit;
            uint16_t fault_code;
        };

        std::vector<ErrorBitMap> error_map = {
            {0, 0x80, 0x0001}, // 负限位保护
            {0, 0x40, 0x0002}, // 正限位保护
            {0, 0x20, 0x0003}, // 超过最大速度
            {0, 0x10, 0x0004}, // 过载
            {0, 0x08, 0x0005}, // 堵转
            {0, 0x04, 0x0006}, // 过温
            {0, 0x02, 0x0007}, // 欠压
            {0, 0x01, 0x0008}, // 过压
            {1, 0x08, 0x0009}, // CAN通信故障
            {1, 0x04, 0x000A}, // 相电流采样故障
            {1, 0x02, 0x000B}, // 位置跃迁过大故障
            {1, 0x01, 0x000C}  // 绝对值编码器故障
        };

        for (const auto& mapping : error_map) {
            uint8_t byte = (mapping.byte == 0) ? error_byte0 : error_byte1;
            if (byte & mapping.bit) {
                any_error = true;
                uint32_t error_code = (0x3 << 24) | (motor_id << 16) | mapping.fault_code;
                if (error_pub_) {
                    std_msgs::msg::Int64 msg;
                    msg.data = static_cast<int64_t>(error_code);
                    error_pub_->publish(msg);
                    RCLCPP_INFO(get_node()->get_logger(), "Published hardware error: 0x%06X", error_code);
                }
                switch (mapping.fault_code) {
                    case 0x0001: error_messages.push_back("负限位保护"); break;
                    case 0x0002: error_messages.push_back("正限位保护"); break;
                    case 0x0003: error_messages.push_back("超过最大速度"); break;
                    case 0x0004: error_messages.push_back("过载"); break;
                    case 0x0005: error_messages.push_back("堵转"); break;
                    case 0x0006: error_messages.push_back("过温"); break;
                    case 0x0007: error_messages.push_back("欠压"); break;
                    case 0x0008: error_messages.push_back("过压"); break;
                    case 0x0009: error_messages.push_back("CAN通信故障"); break;
                    case 0x000A: error_messages.push_back("相电流采样故障"); break;
                    case 0x000B: error_messages.push_back("位置跃迁过大故障"); break;
                    case 0x000C: error_messages.push_back("绝对值编码器故障"); break;
                }
            }
        }

        if (!any_error) {
            uint32_t error_code = (0x3 << 24) | (motor_id << 16) | 0x0100;
            if (error_pub_) {
                std_msgs::msg::Int64 msg;
                msg.data = static_cast<int64_t>(error_code);
                error_pub_->publish(msg);
                RCLCPP_WARN(get_node()->get_logger(), "Published unknown hardware error: 0x%06X", error_code);
            }
            error_messages.push_back("未知错误");
        }

        if (!error_messages.empty()) {
            std::string error_str = "Motor " + std::to_string(motor_id) + " emergency errors: ";
            for (size_t i = 0; i < error_messages.size(); ++i) {
                error_str += error_messages[i];
                if (i < error_messages.size() - 1) {
                    error_str += ", ";
                }
            }
            RCLCPP_ERROR(get_node()->get_logger(), "%s (SEVERE)", error_str.c_str());
        }
        return;
    }

    uint16_t error_code_raw = frame.data[0] | (static_cast<uint16_t>(frame.data[1]) << 8);
    parseAndReportErrorCode(motor_id, error_code_raw, "emergency frame");
}

// === 控制线程：周期性发送多控帧 ===
// 节奏由 control_period_ms_ 决定（xacro 里配 2 ms = 500 Hz），**与 CM 的 update_rate 无关**。
void JxHardware::controlLoop() {
    RCLCPP_INFO(get_node()->get_logger(), "Control thread started (period: %.2f ms)", control_period_ms_);
    auto period = std::chrono::microseconds(static_cast<int>(control_period_ms_ * 1000));
    auto next_time = std::chrono::steady_clock::now() + period;

    while (running_) {
        // Failure is latched inside sendMultiMotorCommand and surfaced by read/write.
        // Keep attempting CSP hold until lifecycle shutdown joins this thread.
        (void)sendMultiMotorCommand();

        // 精确睡眠到下一个时间点
        std::this_thread::sleep_until(next_time);
        next_time += period;

        // 防止累积误差
        const auto now = std::chrono::steady_clock::now();
        if (next_time < now) {
            // 如果落后了，调整到下个周期
            next_time = now + period;
        }
    }
    RCLCPP_INFO(get_node()->get_logger(), "Control thread stopped");
}

} // namespace juxie_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(juxie_ros2_control::JxHardware, hardware_interface::SystemInterface)
