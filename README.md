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

在使用前，确保 CAN 接口已正确配置。硬件接口会自动配置 CAN FD（1Mbps/5Mbps），但需要确保：

1. CAN 接口已连接并可用
2. 具有配置 CAN 接口的权限（可能需要 sudo）

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
