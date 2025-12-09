#include "juxie_ros2_control/jx_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstring>
#include <cerrno>

namespace juxie_ros2_control {

int16_t JxHardware::angleToInt16(double angle) {
    // 将输入角度限制在[-180, 180]度范围内
    double clamped_angle = std::clamp(angle, ANGLE_MIN, ANGLE_MAX);    
    // 计算角度在范围内的比例（0~1）
    double ratio = (clamped_angle - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN);
    
    // 将比例映射到协议定义的计数范围[-32568, 32568]
    int32_t raw_value = RAW_MIN + ratio * (RAW_MAX - RAW_MIN);
    
    // 确保结果在协议范围内（防止计算误差）
    return static_cast<int16_t>(std::clamp(raw_value, (int32_t)RAW_MIN, (int32_t)RAW_MAX));
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

    // 解析核心参数
    can_interface_ = get_param("can_interface", "can0");
    control_period_ms_ = std::stoi(get_param("control_period_ms", "1")); // 默认1ms=1000Hz

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
    running_ = false;

    // 打印初始化信息
    RCLCPP_INFO(get_node()->get_logger(), "JxHardware initialized:");
    RCLCPP_INFO(get_node()->get_logger(), "  CAN Interface: %s", can_interface_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Control Period: %d ms", control_period_ms_);
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

    // 初始化CAN FD接口（启用1M/5M速率）
    if (!initCanFd()) {
        RCLCPP_ERROR(get_node()->get_logger(), "CAN FD initialization failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 使能电机  ///  待定
    // if (!enableMotors()) {
    //     RCLCPP_ERROR(get_node()->get_logger(), "Motor enable failed");
    //     ::close(can_socket_);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

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
    }

    // 5. 启动控制线程
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
    readMotorStates(false); 
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type JxHardware::write(
    const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) {
    // 弧度→角度→电机计数
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        double angle_deg = joint_position_commands_[i] * 180.0 / M_PI;
        raw_position_commands_[i] = angleToInt16(angle_deg);
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

    RCLCPP_INFO(get_node()->get_logger(), "CAN FD initialized: %s (1M/5M)", can_interface_.c_str());
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
    uint8_t data[CANFD_MAX_LEN] = {0};
    uint8_t control_byte = is_first_command_ ? 0xD8 : 0xD0;

    // 填充每个电机的控制包（7字节/个，共8个）
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        int pkg_offset = i * 7;
        data[pkg_offset + 0] = control_byte; // 控制位：ENABLE=1、BRAKE=1、MODE=位置模式
        data[pkg_offset + 1] = (raw_position_commands_[i] >> 8) & 0xFF; // 位置高字节
        data[pkg_offset + 2] = raw_position_commands_[i] & 0xFF;        // 位置低字节
        data[pkg_offset + 3] = 0x00; // TC=0（位置模式无需电流前馈）
        data[pkg_offset + 4] = 0x00;
        data[pkg_offset + 5] = 0x00; // TFC=0
        data[pkg_offset + 6] = 0x00;
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
        RCLCPP_INFO(get_node()->get_logger(), "Sent first command with control byte 0xD8");
        is_first_command_ = false;
    }

    return true;
}
// === 读取电机状态（非阻塞读取+缓存所有反馈）===
bool JxHardware::readMotorStates(bool strict_check) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    struct timeval timeout = {0, 1000}; // 缩短超时到1ms，避免阻塞
    ::setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // 创建临时映射表存储最新反馈
    std::unordered_map<uint8_t, canfd_frame> feedback_map;
    struct canfd_frame frame;
    
    // 读取所有可用的CAN帧
    while (true) {
        ssize_t nbytes = ::read(can_socket_, &frame, sizeof(frame));
        if (nbytes <= 0) {
            break; // 无数据或超时
        }

        // 检查是否是目标电机的反馈帧
        if (frame.can_id >= SINGLE_FEEDBACK_BASE && frame.can_id < SINGLE_FEEDBACK_BASE + 256) {
            uint8_t motor_id = frame.can_id - SINGLE_FEEDBACK_BASE;
            // 只缓存我们关心的电机反馈
            if (std::find(motor_ids_.begin(), motor_ids_.end(), motor_id) != motor_ids_.end()) {
                feedback_map[motor_id] = frame; // 更新最新反馈
            }
        }
    }

    if(feedback_map.size()!=motor_ids_.size() && strict_check )
    {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                            1000, "feedback_map size is,motor ids size is:%d ,%d",feedback_map.size(),motor_ids_.size());
        return false;
    }




    // 处理缓存的反馈数据
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        uint8_t id = motor_ids_[i];
        auto it = feedback_map.find(id);
        
        if (it != feedback_map.end()) {
            // 解析数据
            const canfd_frame& frame = it->second;
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
            }
        } else {
            RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                                1000, "No feedback for motor %d", id);
        }
    }

    return true;
}

// === 控制线程：周期性发送多控帧 ===
void JxHardware::controlLoop() {
    RCLCPP_INFO(get_node()->get_logger(), "Control thread started (period: %d ms)", control_period_ms_);
    auto period = std::chrono::milliseconds(control_period_ms_);
    auto next_time = std::chrono::steady_clock::now() + period;

    while (running_) {
        sendMultiMotorCommand();
        std::this_thread::sleep_until(next_time);
        next_time += period;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Control thread stopped");
}

} // namespace juxie_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(juxie_ros2_control::JxHardware, hardware_interface::SystemInterface)
