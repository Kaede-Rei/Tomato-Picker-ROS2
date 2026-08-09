#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace tomato_picker_eef {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

inline constexpr double kEefCommandMinimum = 0.0;              ///< 归一化命令下界
inline constexpr double kEefCommandMaximum = 1.0;              ///< 归一化命令上界
inline constexpr double kEefMotorOpenPosition = 0.0;           ///< 完全张开电机位置 rad
inline constexpr double kEefMotorClosedPosition = -5.23;       ///< 完全闭合电机位置 rad
inline constexpr double kEefMotorPositionOffset = 0.0;         ///< 位置映射偏移 rad
inline constexpr double kEefMotorPositionScale = -5.23;        ///< 位置映射比例 rad
inline constexpr double kDm4310TorqueConstant = 0.945;         ///< DM4310 扭矩常数
inline constexpr double kEmitVelocityScale = 100.0;            ///< 协议速度缩放
inline constexpr double kEmitCurrentScale = 1000.0;            ///< 协议电流缩放
inline constexpr double kDm4310Rpm = 200.0;                    ///< DM4310 额定转速
inline constexpr double kTorquePosRawMaximum = 10000.0;        ///< Torque_Pos raw 上界
inline constexpr double kPi = 3.14159265358979323846;          ///< 圆周率

/**
 * @brief 达妙 Torque_Pos 命令的 wire 参数
 */
struct TorquePosCommand {
    float target_position{ 0.0F };       ///< 电机目标位置，单位 rad
    std::uint16_t velocity_limit{ 0 };   ///< 速度限制 raw 值
    std::uint16_t current_limit{ 0 };    ///< 电流限制 raw 值
};

/**
 * @brief 一次 homing feedback 对状态机产生的决定
 */
enum class HomingDecision {
    CONTINUE,
    ZERO_AND_FINISH,
    FEEDBACK_FAILURE,
    TIMEOUT,
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 将归一化 EEF 命令映射到电机位置
 * @param command 归一化命令，0 为张开，1 为闭合
 * @return 电机目标位置，单位 rad
 */
inline double command_to_motor_position(double command) noexcept {
    const double clamped = std::clamp(command, kEefCommandMinimum, kEefCommandMaximum);
    return kEefMotorPositionOffset + clamped * kEefMotorPositionScale;
}

/**
 * @brief 将电机位置映射为归一化 EEF 反馈
 * @param motor_position 电机位置，单位 rad
 * @return 归一化位置，0 为张开，1 为闭合
 */
inline double motor_position_to_command(double motor_position) noexcept {
    const double command = (motor_position - kEefMotorPositionOffset) / kEefMotorPositionScale;
    return std::clamp(command, kEefCommandMinimum, kEefCommandMaximum);
}

/**
 * @brief 计算 DM4310 速度限制 raw 值
 * @param rpm 电机额定转速
 * @return 有效的 uint16 raw 值，非法或超过协议范围时返回空
 */
inline std::optional<std::uint16_t> calculate_velocity_limit(double rpm = kDm4310Rpm) noexcept {
    const double raw = rpm / 60.0 * 2.0 * kPi * kEmitVelocityScale;
    if(!std::isfinite(raw) || raw < 0.0 || raw > kTorquePosRawMaximum ||
        raw > static_cast<double>(std::numeric_limits<std::uint16_t>::max())) {
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(raw);
}

/**
 * @brief 将最大夹持扭矩换算为电流限制 raw 值
 * @param max_gripper_torque 最大夹持扭矩，单位 N·m
 * @return 有效的 uint16 raw 值，非法或超过协议范围时返回空
 */
inline std::optional<std::uint16_t> calculate_current_limit(double max_gripper_torque) noexcept {
    const double raw = max_gripper_torque / kDm4310TorqueConstant * kEmitCurrentScale;
    if(!std::isfinite(raw) || raw < 0.0 || raw > kTorquePosRawMaximum ||
        raw > static_cast<double>(std::numeric_limits<std::uint16_t>::max())) {
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(raw);
}

/**
 * @brief 生成可直接传给 control_pos_force() 的命令
 * @param position_command 归一化位置，0 为张开，1 为闭合
 * @param max_gripper_torque 最大夹持扭矩，单位 N·m
 * @return 参数全部有效时返回 Torque_Pos 命令
 */
inline std::optional<TorquePosCommand> make_torque_pos_command(
    double position_command,
    double max_gripper_torque) noexcept {
    if(!std::isfinite(position_command) || position_command < kEefCommandMinimum ||
        position_command > kEefCommandMaximum) {
        return std::nullopt;
    }
    const auto velocity = calculate_velocity_limit();
    const auto current = calculate_current_limit(max_gripper_torque);
    const double position = command_to_motor_position(position_command);
    if(!velocity || !current || !std::isfinite(position)) return std::nullopt;
    return TorquePosCommand{ static_cast<float>(position), *velocity, *current };
}

/**
 * @brief 不访问硬件的 EEF homing 判定状态机
 */
class EefHomingStateMachine final {
public:
    using Clock = std::chrono::steady_clock;

    /**
     * @brief 构造 homing 判定状态机
     * @param torque_threshold 机械止挡扭矩阈值，单位 N·m
     * @param timeout 最大 homing 时间
     */
    EefHomingStateMachine(double torque_threshold, std::chrono::duration<double> timeout) noexcept
        : torque_threshold_(torque_threshold), timeout_(timeout) {
    }

    /**
     * @brief 开始一次 homing
     * @param now 当前单调时钟
     */
    void start(Clock::time_point now) noexcept {
        started_at_ = now;
        started_ = true;
    }

    /**
     * @brief 根据新反馈判定下一步
     * @param torque 有效反馈扭矩；无值表示刷新失败
     * @param now 当前单调时钟
     * @return 继续、寻零完成、反馈失败或超时
     */
    HomingDecision observe(std::optional<double> torque, Clock::time_point now) const noexcept {
        if(!started_ || !std::isfinite(torque_threshold_) || torque_threshold_ <= 0.0 ||
            !std::isfinite(timeout_.count()) || timeout_.count() <= 0.0) {
            return HomingDecision::FEEDBACK_FAILURE;
        }
        if(now - started_at_ >= timeout_) return HomingDecision::TIMEOUT;
        if(!torque || !std::isfinite(*torque)) return HomingDecision::FEEDBACK_FAILURE;
        if(*torque > torque_threshold_) return HomingDecision::ZERO_AND_FINISH;
        return HomingDecision::CONTINUE;
    }

private:
    double torque_threshold_{ 0.0 };         ///< 机械止挡扭矩阈值，单位 N·m
    std::chrono::duration<double> timeout_;  ///< 最大 homing 时间
    Clock::time_point started_at_{};         ///< homing 起始时刻
    bool started_{ false };                  ///< 是否已经开始 homing
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace tomato_picker_eef
