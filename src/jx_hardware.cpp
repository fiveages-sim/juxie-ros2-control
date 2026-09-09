#include "juxie_ros2_control/jx_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <cctype>
#include <cstdio>
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

const char* JxHardware::controlModeName() const {
    const bool has_csv = hasVelocityControl();
    const bool has_csp = std::find(motor_control_modes_.begin(), motor_control_modes_.end(),
                                   ControlMode::CSP_POSITION) != motor_control_modes_.end();
    if (has_csv && has_csp) {
        return "mixed CSP/CSV";
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
    const ControlMode configured_mode =
        default_mode == "velocity" ? ControlMode::CSV_VELOCITY : ControlMode::CSP_POSITION;
    if (default_mode != "position" && default_mode != "velocity") {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'control_mode' must be 'position' or 'velocity', got '%s'",
                     default_mode.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    motor_control_modes_.assign(motor_ids_.size(), configured_mode);

    std::string velocity_ids = get_param("velocity_motor_ids", "");
    velocity_ids.erase(std::remove(velocity_ids.begin(), velocity_ids.end(), '['), velocity_ids.end());
    velocity_ids.erase(std::remove(velocity_ids.begin(), velocity_ids.end(), ']'), velocity_ids.end());
    std::istringstream iss_velocity_ids(velocity_ids);
    while (std::getline(iss_velocity_ids, token, ',')) {
        if (token.find_first_not_of(" \t") == std::string::npos) {
            continue;
        }
        const int parsed_id = std::stoi(token);
        if (parsed_id < 1 || parsed_id > 127)
            throw std::invalid_argument("velocity_motor_ids must be in [1,127]");
        const uint8_t id = static_cast<uint8_t>(parsed_id);
        const auto it = std::find(motor_ids_.begin(), motor_ids_.end(), id);
        if (it == motor_ids_.end()) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "velocity_motor_ids contains motor %u not present in motor_ids", id);
            return hardware_interface::CallbackReturn::ERROR;
        }
        motor_control_modes_[static_cast<size_t>(std::distance(motor_ids_.begin(), it))] =
            ControlMode::CSV_VELOCITY;
    }

    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        if (motor_control_modes_[i] == ControlMode::CSV_VELOCITY &&
            std::none_of(info_.joints[i].command_interfaces.begin(), info_.joints[i].command_interfaces.end(),
                         [](const auto& interface) { return interface.name == "velocity"; }))
            throw std::invalid_argument("CSV joint must declare a velocity command interface");
    }
    max_velocity_rpm_ = std::stoi(get_param("max_velocity_rpm", "32767"));
    if (max_velocity_rpm_ <= 0 || max_velocity_rpm_ > 32767) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Parameter 'max_velocity_rpm' must be in [1,32767], got %d",
                     max_velocity_rpm_);
        return hardware_interface::CallbackReturn::ERROR;
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
    raw_position_commands_.resize(motor_count, 0);
    raw_velocity_commands_.resize(motor_count, 0);
    target_raw_position_commands_.resize(motor_count, 0);
    last_sent_raw_commands_.resize(motor_count, 0);
    last_sent_deltas_.resize(motor_count, 0);

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
hardware_interface::CallbackReturn JxHardware::on_activate(
    const rclcpp_lifecycle::State & /* previous_state */) {
    RCLCPP_INFO(get_node()->get_logger(), "Activating JxHardware...");
    locked_feedback_len_ = 0;
    is_first_command_ = true;

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

    if (hasVelocityControl() && locked_feedback_len_ != 12) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "CSV velocity mode requires 12-byte feedback; detected %u bytes",
                     locked_feedback_len_);
        ::close(can_socket_);
        can_socket_ = -1;
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 初始化命令缓冲区（从当前反馈起步，避免首帧跃迁）
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        joint_position_commands_[i] = joint_positions_[i]; // 弧度
        double angle_deg = joint_positions_[i] * 180.0 / M_PI; // 弧度→角度
        raw_position_commands_[i] = angleToInt16(angle_deg); // 角度→电机计数
        target_raw_position_commands_[i] = raw_position_commands_[i];
        last_sent_raw_commands_[i] = raw_position_commands_[i];
        last_sent_deltas_[i] = 0;
        joint_velocity_commands_[i] = 0.0;
        raw_velocity_commands_[i] = 0;
    }

    // 启动控制线程
    running_ = true;
    control_thread_ = std::thread(&JxHardware::controlLoop, this);

    RCLCPP_INFO(get_node()->get_logger(), "JxHardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// === 停用硬件（停止线程+关闭CAN）===
hardware_interface::CallbackReturn JxHardware::on_deactivate(
    const rclcpp_lifecycle::State & /* previous_state */) {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating JxHardware...");

    running_ = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    if (can_socket_ >= 0) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            std::fill(joint_velocity_commands_.begin(), joint_velocity_commands_.end(), 0.0);
            std::fill(raw_velocity_commands_.begin(), raw_velocity_commands_.end(), 0);
        }
        if (hasVelocityControl()) {
            (void)sendMultiMotorCommand();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ::close(can_socket_);
        can_socket_ = -1;
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
    }
    RCLCPP_INFO(get_node()->get_logger(), "Exported %zu command interfaces (%s)",
                command_interfaces.size(), controlModeName());
    return command_interfaces;
}

// === ROS2 Control 读写接口 ===
hardware_interface::return_type JxHardware::read(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) {
    if (!readMotorStates(false))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "motor error ,return");
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

    // Validate before conversion to integer protocol units.
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        const double command = motor_control_modes_[i] == ControlMode::CSV_VELOCITY
            ? joint_velocity_commands_[i] : joint_position_commands_[i];
        if (!std::isfinite(command)) {
            std::fill(raw_velocity_commands_.begin(), raw_velocity_commands_.end(), 0);
            RCLCPP_ERROR(get_node()->get_logger(), "Non-finite command for motor %u", motor_ids_[i]);
            return hardware_interface::return_type::ERROR;
        }
    }
    // 更新最新目标；实际下发由独立控制线程完成。
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        if (motor_control_modes_[i] == ControlMode::CSP_POSITION) {
            const double angle_deg = joint_position_commands_[i] * 180.0 / M_PI;
            target_raw_position_commands_[i] = angleToInt16(angle_deg);
        } else {
            const double motor_rpm = joint_velocity_commands_[i] * 60.0 /
                                     (2.0 * M_PI) * gear_ratios_[i];
            const long rounded = std::lround(std::clamp(
                motor_rpm, -static_cast<double>(max_velocity_rpm_),
                static_cast<double>(max_velocity_rpm_)));
            raw_velocity_commands_[i] = static_cast<int16_t>(rounded);
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

// === 电机使能 ===
bool JxHardware::enableMotors() {
    uint8_t enable_cmd[] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
    for (uint8_t id : motor_ids_) {
        struct canfd_frame frame;
        memset(&frame, 0, sizeof(frame));
        frame.can_id = static_cast<canid_t>(0x600 + id);
        frame.len = 8;
        memcpy(frame.data, enable_cmd, 8);
        if (::write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Enable motor %d failed", id);
            ::close(can_socket_);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
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

    uint8_t data[CANFD_MAX_LEN] = {0};
    // 填充每个电机的控制包（7字节/个，共8个）
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        int pkg_offset = i * 7;
        const bool is_position = motor_control_modes_[i] == ControlMode::CSP_POSITION;
        const uint8_t control_byte = locked_feedback_len_ == 7
            ? (is_first_command_ ? 0xD8 : 0xD0)
            : (is_position
                ? (is_first_command_ ? CTRL_WORD_CSP_CLEAR : CTRL_WORD_CSP)
                : (is_first_command_ ? CTRL_WORD_CSV_CLEAR : CTRL_WORD_CSV));
        const int16_t raw_command = is_position
            ? raw_position_commands_[i] : raw_velocity_commands_[i];
        const uint16_t encoded = static_cast<uint16_t>(raw_command);
        data[pkg_offset + 0] = control_byte;
        data[pkg_offset + 1] = (encoded >> 8) & 0xFF;
        data[pkg_offset + 2] = encoded & 0xFF;
        data[pkg_offset + 3] = 0x00; // 目标参数2，CSP模式下不使用
        data[pkg_offset + 4] = 0x00;
        data[pkg_offset + 5] = 0x00; // 目标前馈参数，CSP模式下不使用
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
    if (::write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Multi-motor command failed: %s", strerror(errno));
        return false;
    }
    
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
    // 存储错误帧的映射表
    std::unordered_map<uint8_t, canfd_frame> error_frames_map;

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
                    error_frames_map[motor_id] = frame;
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
    }

    // 处理缓存的反馈数据
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        uint8_t id = motor_ids_[i];
        auto it = feedback_map.find(id);
        
        if (it != feedback_map.end()) {
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
                uint8_t mode_feedback = frame.data[10];
                uint8_t status_byte = frame.data[11];
                bool enable = (status_byte & 0x80) != 0;
                bool error = (status_byte & 0x20) != 0 || error_code != 0;

                if (!enable) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                       5000, "Motor %d is not enabled", id);
                }
                const uint8_t expected_mode = motor_control_modes_[i] == ControlMode::CSV_VELOCITY
                    ? 0x04 : 0x03;
                if (mode_feedback != expected_mode) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                        5000, "Motor %d mode feedback is 0x%02X, expected 0x%02X (%s)",
                                        id, mode_feedback, expected_mode,
                                        motor_control_modes_[i] == ControlMode::CSV_VELOCITY
                                            ? "CSV velocity" : "CSP position");
                }
                if (error) {
                    if (error_code != 0) {
                        parseAndReportErrorCode(id, error_code, "feedback frame");
                    }
                    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                        1000, "Motor %d has error, error code: 0x%04X, status: 0x%02X",
                                        id, error_code, status_byte);
                    running_ = false;
                    return false;
                }
            } else if (protocol_len == 7) {
                if (frame.len < 7) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                       1000, "Feedback for motor %d has insufficient data length: %d (<7)",
                                       id, frame.len);
                    continue;
                }
                int16_t raw_pos = (frame.data[0] << 8) | frame.data[1];
                double angle_deg = int16ToAngle(raw_pos);
                joint_positions_[i] = angle_deg * M_PI / 180.0;

                int16_t raw_vel = (frame.data[2] << 8) | frame.data[3];
                joint_velocities_[i] = raw_vel * 2 * M_PI / 60.0 / gear_ratios_[i];

                joint_efforts_[i] = 0.0;
                uint8_t status_byte = frame.data[6];
                bool enable = (status_byte & 0x20) != 0;
                bool error = (status_byte & 0x08) != 0;

                if (!enable) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                       5000, "Motor %d is not enabled", id);
                }
                if (error) {
                    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                        1000, "Motor %d has error, error code: 0x%02X", id, status_byte);
                    running_ = false;
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
            RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                1000, "No feedback for motor %d", id);
        }
    }

    return true;
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
void JxHardware::controlLoop() {
    RCLCPP_INFO(get_node()->get_logger(), "Control thread started (period: %.2f ms)", control_period_ms_);
    auto period = std::chrono::microseconds(static_cast<int>(control_period_ms_ * 1000));
    auto next_time = std::chrono::steady_clock::now() + period;

    while (running_) {
        sendMultiMotorCommand();
        
        // 精确睡眠到下一个时间点
        std::this_thread::sleep_until(next_time);
        next_time += period;
        
        // 防止累积误差
        auto now = std::chrono::steady_clock::now();
        if (next_time < now) {
            // 如果落后了，调整到下个周期
            next_time = now + period;
    }
}
    RCLCPP_INFO(get_node()->get_logger(), "Control thread stopped");

} // namespace juxie_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(juxie_ros2_control::JxHardware, hardware_interface::SystemInterface)
}
