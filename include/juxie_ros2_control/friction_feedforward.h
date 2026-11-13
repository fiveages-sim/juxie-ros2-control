#pragma once

#include <algorithm>
#include <cmath>

namespace juxie_ros2_control {
// 摩擦前馈电流（mA，带符号）：sign·min(f_c + f_v·|v|, 上限)。
//
// 为什么单独抽出来（而不是写在 sendMultiMotorCommand 里）：
//   腰部升降这类低速运动 |q̇_d| ≈ 0.02 rad/s 低于摩擦死区 0.03 rad/s，而原实现
//   用「实测速度」判定是否补摩擦 —— 关节还卡着时 |q̇| ≈ 0，于是整段升降里摩擦前馈
//   都不生效。位置误差只能被阻抗刚度一点点放大到 f_c/Kp（j1 ≈ 18 mrad ≈ 1°），
//   到位后才突然滑动：实测就是 ~2.5 s 周期的粘滑锯齿，躯干因此前后摆 1~2°。
//
// 三条分支：
//   ① |v_ff| ≥ eps（已经滑起来）：与原实现完全一致（方向优先取参考速度）；
//   ② |v_ff| < eps 但**指令在动**（reference 模式）：按 |q̇_d| / eps 的比例先补，
//      既填掉锯齿，又不会在刚起动的瞬间整额投入；
//   ③ 指令也不动（HOLD）：补偿为 0，保持原死区行为（避免 1 rpm 量化自激）。
//
// 参数（与调用处一一对应）：
//   measured_velocity_rad_s   实测关节速度 q̇
//   reference_velocity_rad_s  上层速度指令 q̇_d（CST/CSV 的 velocity 命令口）
//   coulomb_ma / viscous_ma_s_rad  辨识得到的 f_c / f_v
//   vel_eps_rad_s             速度死区；0 = 关闭死区（此时行为与原实现一致）
//   max_ma                    幅值上限；≤0 = 不限
//   use_reference_velocity    friction_ff_velocity == "reference"（或 reference_ramp）
//   fill_deadband             friction_ff_velocity == "reference_ramp" 时才为 true：
//                             开启第 ② 条分支。默认关闭 = 与原实现逐字节一致。
//   stiction_ma               静摩擦 f_s（> f_c 时生效；默认 0 = 关闭）
//   stribeck_vel_rad_s        Stribeck 速度 v_s：按**实测**速度衰减 (f_s−f_c)·e^-(|q̇|/v_s)²
//   ramp_vel_rad_s            死区内补摩擦的**斜坡宽度**（0 = 沿用 vel_eps_rad_s）。
//                             死区 vel_eps 是“用什么速度判方向”的门槛，斜坡宽度是
//                             “补多少”的门槛，两者目的不同，不该共用一个数（见下）。
//
// 为什么要 f_s：只补 f_c（动能摩擦）时，关节卡住那一刻需要的是 f_s > f_c，于是得先靠弹簧
// 再压出 (f_s−f_c)/Kp 才滑动，形成「停一下→窜一下」的极限环（2026-10-29 W2 实测 ~1.6 Hz，
// j1/j2/j3 同步，q̇ 在 0 与 2×均值之间来回切换）。补上 f_s 后关节不再进入粘住状态。
//
// 为什么要 ramp_vel 而不是拿 vel_eps 当斜坡：死区是**关节空间**的门槛，但实际低速升降给
// 的指令是**末端空间**的（比如 z 方向 6 mm/s）。同一个 6 mm/s，j1 只有 0.0087 rad/s、j2 有
// 0.0194 rad/s —— 若 v_eps=0.015，则 j1 只能补到 58%、j3 补到 71%、j2 补满。三轴补不齐就
// 不是“整体下沉”，而是姿态前后摆。把斜坡宽度缩到 0.0015 rad/s 后，任何**真的在走**的指令
// 都能补满，而指令真为 0（HOLD）时仍然是 0，不会自激。
inline double frictionFeedforwardMa(double measured_velocity_rad_s,
                                    double reference_velocity_rad_s,
                                    double coulomb_ma,
                                    double viscous_ma_s_rad,
                                    double vel_eps_rad_s,
                                    double max_ma,
                                    bool use_reference_velocity,
                                    bool fill_deadband = false,
                                    double stiction_ma = 0.0,
                                    double stribeck_vel_rad_s = 0.0,
                                    double ramp_vel_rad_s = 0.0,
                                    double min_vel_rad_s = 0.0)
{
    const double eps = std::max(vel_eps_rad_s, 0.0);
    const double abs_vd = std::abs(reference_velocity_rad_s);
    const double abs_v = std::abs(measured_velocity_rad_s);

    const auto magnitude = [&](double speed) {
        double value = coulomb_ma + viscous_ma_s_rad * speed;
        if (stiction_ma > coulomb_ma && stribeck_vel_rad_s > 0.0) {
            // 卡住（|q̇| → 0）时按静摩擦出力，滑起来（|q̇| ≫ v_s）后只剩 f_c。
            const double u = abs_v / stribeck_vel_rad_s;
            value += (stiction_ma - coulomb_ma) * std::exp(-u * u);
        }
        if (max_ma > 0.0) value = std::min(value, max_ma);
        return value;
    };

    // ① 原来就有的路径：滑起来后按「参考优先」的符号全额补偿。
    const double v_ff = (use_reference_velocity && abs_vd >= eps)
                            ? reference_velocity_rad_s
                            : measured_velocity_rad_s;
    if (std::abs(v_ff) >= eps) {
        return (v_ff > 0.0 ? 1.0 : -1.0) * magnitude(std::abs(v_ff));
    }

    // ② 可选（reference_ramp）：指令在动、但关节还卡在静摩擦里时按比例先补。
    if (fill_deadband && use_reference_velocity && eps > 0.0 && abs_vd > 1e-6) {
        // 噪声死区：|v_d| 小于它 = 实际上没在动，补偿直接归 0。
        // 没有这一条，下面 scale = |v_d|/ramp 一饱和就是满幅，而 ramp 只有
        // 0.0015 ⇒ 速度噪声会让 ±f_s 满幅乱翻号（2026-11-11 实测 Σd·ff
        // 峰峰 2357 mA ≈ ±40 N·m，位置不动纯浪费，且余量为零）。
        if (min_vel_rad_s > 0.0 && abs_vd < min_vel_rad_s) {
            return 0.0;
        }
        // 斜坡宽度与死区解耦：死区判方向，斜坡判补多少。
        const double ramp = (ramp_vel_rad_s > 0.0) ? ramp_vel_rad_s : eps;
        const double scale = std::clamp(abs_vd / ramp, 0.0, 1.0);
        return (reference_velocity_rad_s > 0.0 ? 1.0 : -1.0) * scale * magnitude(abs_vd);
    }

    // ③ 指令也不动：不补。
    return 0.0;
}

}  // namespace juxie_ros2_control
