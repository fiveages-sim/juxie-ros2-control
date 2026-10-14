# Juxie ROS2 Control

ROS2 Control 硬件接口，用于通过 CAN FD 通信控制 JX 电机（循环同步位置模式）。

## 功能特性

- 支持 CAN FD 通信（1Mbps 仲裁段，5Mbps 数据段）
- 支持最多 8 个电机同时控制
- 循环同步位置（CSP）控制模式
- 提供位置、速度状态反馈
- 支持位置命令控制

## 硬件接口

### 命令接口（Command Interfaces）
- `position`: 关节位置命令（弧度）

### 状态接口（State Interfaces）
- `position`: 关节实际位置（弧度）
- `velocity`: 关节实际速度（弧度/秒）

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-up-to juxie_ros2_control --symlink-install
source install/setup.bash
```

## 配置参数

在 ROS2 Control 配置中需要设置以下硬件参数：

- `can_interface` (string, 默认: `"can0"`): CAN 接口名称
- `motor_ids` (string, 必需): 电机 ID 列表，格式为 `"1,2,3"` 或 `"[1,2,3]"`
- `control_period_ms` (int, 默认: `1`): 控制周期（毫秒），默认 1ms = 1000Hz
- `max_position_step_nct` (int, 默认: `25`): 每个发送周期允许的最大位置增量（nct）。
- `max_position_accel_nct` (int, 默认: `5`): 相邻两帧指令速度(delta)的最大变化量（nct/cycle²），用于加减速兜底；

## 使用示例

### 1. 在 URDF/Xacro 中配置硬件接口

在您的机器人描述文件中添加 ROS2 Control 配置：

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 定义关节接口 -->
  <ros2_control name="juxie_system" type="system">
    <hardware>
      <plugin>juxie_ros2_control/JxHardware</plugin>
      <!-- CAN 接口配置 -->
      <param name="can_interface">can0</param>
      <!-- 电机 ID 列表（最多 8 个） -->
      <param name="motor_ids">1,2,3</param>
      <!-- 控制周期（毫秒） -->
      <param name="control_period_ms">1</param>
      <!-- 单帧最大位置增量（nct），防位置跃迁 -->
      <param name="max_position_step_nct">25</param>
      <param name="max_position_accel_nct">5</param>
    </hardware>

    <!-- 关节接口定义 -->
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>

    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>

    <joint name="joint3">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

</robot>
```

### 2. 配置 CAN 接口

#### 配置步骤

1. **编辑 sudoers 文件**

```bash
sudo visudo
```

2. **添加以下行**（将 `your_username` 替换为你的实际用户名，将 `can0` 为默认配置的设备名称）

```
your_username ALL=(ALL) NOPASSWD: /sbin/ip link set can0 down, /sbin/ip link set can0 up type can bitrate * dbitrate * fd on

```

**示例**（如果用户名为 `fa`，设备名称为 `can0`）：
```
fa ALL=(ALL) NOPASSWD: /sbin/ip link set can0 down, /sbin/ip link set can0 up type can bitrate * dbitrate * fd on
```
### 3. 启动控制器

使用标准的 ROS2 Control 控制器（如 `joint_trajectory_controller`）：

```bash
# 启动机器人描述
ros2 launch your_robot_description display.launch.py

# 启动控制器管理器
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller joint_trajectory_controller

# 激活控制器
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state joint_trajectory_controller active
```

### 4. 发送位置命令

```bash
# 使用命令行发送位置命令
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['joint1', 'joint2', 'joint3'], \
    points: [{positions: [0.5, -0.5, 1.0], time_from_start: {sec: 2, nanosec: 0}}]}"
```

## 注意事项

1. **电机 ID 顺序**: `motor_ids` 参数中的 ID 顺序必须与 URDF 中定义的关节顺序一致
2. **角度范围**: 电机角度范围为 -180° 到 +180°（-π 到 +π 弧度）
3. **权限要求**: CAN 接口配置需要 root 权限，确保运行节点时具有相应权限
4. **CAN FD 支持**: 确保您的 CAN 适配器支持 CAN FD 模式
5. **控制周期**: 默认 1ms 控制周期，可根据实际需求调整

## 故障排除

- **CAN 接口初始化失败**: 检查 CAN 接口名称是否正确，接口是否已连接
- **电机无响应**: 检查电机 ID 配置是否正确，CAN 总线连接是否正常
- **权限错误**: 确保具有配置 CAN 接口的权限，可能需要使用 sudo 或配置 udev 规则

## 实验性 body_joint4 力矩接口

默认关闭。`joint4_cst_enabled=true` 时增加 joint4 的 `effort`（输出端 Nm）与 `cst_enable` 命令接口，其他关节保持 CSP。只支持明确选择的 `canfd_iq_ma` 协议适配，必须提供标定换算及限值；这不是未经确认的直接 Nm 报文。详见仓库 `controller/ocs2_wbc_controller/config/joint4_impedance.md`。


## 电流环线程与故障保持

- `write()` 校验完整命令后，在互斥锁内提交位置/速度快照；发送线程不读取命令接口裸指针。
- `feedback_timeout_sec` 默认为 `0.02` 秒，必须为正有限数。发送线程按 steady clock 检查每轴最后健康反馈；CM 不再调用 `read()` 且发送线程能够取得 `state_mutex_` 时，超时轴会锁存 CSP 请求；如果该锁被阻塞的读线程持有，本检查也会等待，不能替代硬件看门狗。原 `feedback_timeout_cycles` 检查仍保留，两者任一触发即锁存。
- CAN 瞬时错误（EAGAIN/EWOULDBLOCK、ENOBUFS、EINTR）按正常周期重试，连续失败达 100 ms 或遇到其他错误才锁存故障，由 `read()` / `write()` 返回 ERROR；后续发送尝试仅使用 CSP 保持目标，直到生命周期停用。发送成功不等于电机已确认切换模式。
- 锁存位置独立保存，停用及重新激活均不覆盖；仅重新初始化解除锁存。
- `current_loop_safety_test` 使用本地 datagram socket 检查实际驱动的命令、反馈和发送故障路径，不连接真实 CAN。
