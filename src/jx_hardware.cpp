#include "juxie_ros2_control/jx_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>

namespace juxie_ros2_control {

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

    const auto get_param = [&](const std::string& name, const std::string& default_val) {
        auto it = info_.hardware_parameters.find(name);
        return it != info_.hardware_parameters.end() ? it->second : default_val;
    };
    error_pub_ = get_node()->create_publisher<std_msgs::msg::Int64>("/Base_HardwareError", 10);
    RCLCPP_INFO(get_node()->get_logger(), "Error publisher created on /Base_HardwareError");
    // 解析核心参数
    can_interface_ = get_param("can_interface", "can0");
    control_period_ms_ = std::stof(get_param("control_period_ms", "1")); // 默认1ms=1000Hz

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
    while (std::getline(iss_ids, token, ',')) {
        motor_ids_.push_back(static_cast<uint8_t>(std::stoi(token)));
    }

    // 初始化数据存储
    size_t motor_count = motor_ids_.size();
    joint_positions_.resize(motor_count, 0.0);
    joint_velocities_.resize(motor_count, 0.0);
    joint_efforts_.resize(motor_count, 0.0);
    joint_position_commands_.resize(motor_count, 0.0);
    raw_position_commands_.resize(motor_count, 0);
    
    // 插值相关变量
    prev_raw_position_commands_.resize(motor_count, 0);
    next_raw_position_commands_.resize(motor_count, 0);
    last_sent_raw_commands_.resize(motor_count, 0);
    interpolation_alpha_ = 0.0;
    alpha_increment_ = 0.0;
    
    running_ = false;
    is_first_command_ = true;

    // 打印初始化信息
    RCLCPP_INFO(get_node()->get_logger(), "JxHardware initialized:");
    RCLCPP_INFO(get_node()->get_logger(), "  CAN Interface: %s", can_interface_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Control Period: %.2f ms", control_period_ms_);
    RCLCPP_INFO(get_node()->get_logger(), "  Motor IDs (count: %zu):", motor_ids_.size());

    for (uint8_t id : motor_ids_) {
        RCLCPP_INFO(get_node()->get_logger(), "    - %d", id);
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
}

// === 激活硬件（初始化CAN+使能电机+获取初始状态）===
hardware_interface::CallbackReturn JxHardware::on_activate(
    const rclcpp_lifecycle::State & /* previous_state */) {
    RCLCPP_INFO(get_node()->get_logger(), "Activating JxHardware...");
    locked_feedback_len_ = 0;

    // 初始化CAN FD接口（启用1M/5M速率）
    if (!initCanFd()) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN FD initialization failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 发送同步帧→获取初始状态（仅初始化时发送一次）
    sendSyncFrame();
    if (!readMotorStates(true)) { 
        RCLCPP_ERROR(get_node()->get_logger(), "Incomplete motor state initialization");
        ::close(can_socket_);
        return hardware_interface::CallbackReturn::ERROR;
    }   
    
    // 初始化命令缓冲区
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        joint_position_commands_[i] = joint_positions_[i]; // 弧度
        double angle_deg = joint_positions_[i] * 180.0 / M_PI; // 弧度→角度
        raw_position_commands_[i] = angleToInt16(angle_deg); // 角度→电机计数
        
        // 初始化插值相关变量
        prev_raw_position_commands_[i] = raw_position_commands_[i];
        next_raw_position_commands_[i] = raw_position_commands_[i];
        last_sent_raw_commands_[i] = raw_position_commands_[i];
    }
    
    interpolation_alpha_ = 0.0;
    alpha_increment_ = 0.0;

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

    // 停止控制线程
    if (running_) {
        running_ = false;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
    }

    // 关闭CAN FD接口
    ::close(can_socket_);

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
        // 位置命令接口（弧度）
        command_interfaces.push_back(
            std::make_shared<hardware_interface::CommandInterface>(
                joint_name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
    }
    RCLCPP_INFO(get_node()->get_logger(), "Exported %zu command interfaces (position)", command_interfaces.size());
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
    const rclcpp::Time & /* time */, const rclcpp::Duration &period) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 更新插值相关变量
    // 上一次实际发送的命令作为插值起点
    prev_raw_position_commands_ = last_sent_raw_commands_;
    
    // 计算新的目标命令
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        double angle_deg = joint_position_commands_[i] * 180.0 / M_PI;
        next_raw_position_commands_[i] = angleToInt16(angle_deg);
    }
    
    // 更新插值系数增量
    // 控制线程运行在1000Hz，period是ROS2控制器的更新周期
    double hw_period = period.seconds();
    if (hw_period > 0) {
        // 使用浮点数除法
        alpha_increment_ = (control_period_ms_ / 1000.0) / hw_period;
    } else {
        alpha_increment_ = 1.0;
    }
    
    // 重置插值系数为0，开始新的插值周期
    interpolation_alpha_ = 0.0;
    
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
        return false;
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (::bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN bind failed: %s", strerror(errno));
        ::close(can_socket_);
        return false;
    }

    // 启用CAN FD模式
    int enable_fd = 1;
    if (::setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN FD enable failed: %s", strerror(errno));
        ::close(can_socket_);
        return false;
    }

    // 将 socket 设为非阻塞，避免 read() 在无数据时阻塞到超时，造成 CM 实时循环 Overrun
    int flags = ::fcntl(can_socket_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN set non-blocking failed: %s", strerror(errno));
        ::close(can_socket_);
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
    
    // 更新插值系数
    interpolation_alpha_ += alpha_increment_;
    if (interpolation_alpha_ > 1.0) {
        interpolation_alpha_ = 1.0;
    }
    
    // 线性插值生成当前帧命令
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        raw_position_commands_[i] = prev_raw_position_commands_[i] + 
                                   interpolation_alpha_ * (next_raw_position_commands_[i] - prev_raw_position_commands_[i]);
        last_sent_raw_commands_[i] = raw_position_commands_[i];
    }
    
    uint8_t data[CANFD_MAX_LEN] = {0};
    // 根据锁定的反馈帧长度选择控制字协议：
    // 7字节协议：首帧0xD8，后续0xD0；12字节协议：CSP控制字（首帧带清错位）
    uint8_t control_byte = 0;
    if (locked_feedback_len_ == 7) {
        control_byte = is_first_command_ ? 0xD8 : 0xD0;
    } else {
        control_byte = is_first_command_ ? CTRL_WORD_CSP_CLEAR : CTRL_WORD_CSP;
    }

    // 填充每个电机的控制包（7字节/个，共8个）
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        int pkg_offset = i * 7;
        // === 新协议适配 START: 单个控制分包与新版单轴控制报文保持一致 ===
        data[pkg_offset + 0] = control_byte; // 控制位：ENABLE=1、BRAKE=1、MODE=CSP位置模式
        data[pkg_offset + 1] = (raw_position_commands_[i] >> 8) & 0xFF; // 位置高字节
        data[pkg_offset + 2] = raw_position_commands_[i] & 0xFF;        // 位置低字节
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
        RCLCPP_INFO(get_node()->get_logger(), "Sent first CSP command with control byte 0x%02X", control_byte);
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
                joint_velocities_[i] = raw_vel * 2 * M_PI / 60.0;

                int16_t raw_current = (frame.data[4] << 8) | frame.data[5];
                joint_efforts_[i] = static_cast<double>(raw_current); // 新协议反馈Iq电流，单位mA

                uint16_t error_code = (frame.data[6] << 8) | frame.data[7];
                uint8_t mode_feedback = frame.data[10];
                uint8_t status_byte = frame.data[11];
                bool enable = (status_byte & 0x80) != 0;
                bool error = (status_byte & 0x20) != 0 || error_code != 0;

                if (!enable) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                       5000, "Motor %d is not enabled", id);
                }
                if (mode_feedback != 0x03) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                        5000, "Motor %d mode feedback is 0x%02X, expected CSP(0x03)",
                                        id, mode_feedback);
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
                joint_velocities_[i] = raw_vel * 2 * M_PI / 60.0;

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
        {0x0004, 0x0003, "过温报错"},
        {0x0008, 0x0004, "堵转"},
        {0x0010, 0x0005, "过载"},
        {0x0020, 0x0006, "电流采样错误"},
        {0x0040, 0x0007, "正限位保护"},
        {0x0080, 0x0008, "负限位保护"},
        {0x0100, 0x0009, "编码器通信超时"},
        {0x0200, 0x000A, "电机超过最大速度"},
        {0x0400, 0x000B, "上电电角度初始化失败"},
        {0x1000, 0x000C, "位置误差过大"},
        {0x2000, 0x000D, "编码器故障"}
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

    uint16_t error_code_raw = (frame.data[0] << 8) | frame.data[1];
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
