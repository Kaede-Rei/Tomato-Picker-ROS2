#include "tomato_picker_eef/damiao_eef_driver.hpp"

#include "dm_hw/damiao.hpp"
#include "serial_arm_protocol_damiao_usb2can/bus.hpp"
#include "tomato_picker_eef/eef_model.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace tomato_picker_eef {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //

namespace {

constexpr double kKpApr = 100.0;                              ///< EEF 固定 KP_APR
constexpr std::chrono::milliseconds kHomingPollPeriod{ 10 };  ///< homing 反馈周期
constexpr std::uint32_t kCanIdMaximum = 0x7FF;                ///< 标准 CAN ID 上界
constexpr std::uint32_t kCanFilterMask = 0x7FF;               ///< EEF CAN 精确过滤掩码
constexpr std::uint32_t kEefMotorId = 0x07;                   ///< 固定 EEF 电机 ID
constexpr std::uint32_t kEefMasterId = 0x17;                  ///< 固定 EEF Master ID

/**
 * @brief 从 damiao_eef.yaml 读取的 EEF 硬件配置
 */
struct DriverConfig {
    std::string bus;                          ///< 共享 CAN 通道名
    std::string serial_port;                  ///< USB2CAN 串口
    int baudrate{ 0 };                        ///< 串口波特率
    std::uint32_t motor_id{ 0 };              ///< EEF 电机 ID
    std::uint32_t master_id{ 0 };             ///< EEF Master ID
    double max_gripper_torque{ 0.0 };         ///< 最大夹持扭矩 N·m
    double homing_velocity{ 0.0 };            ///< homing 速度 rad/s
    double homing_torque_threshold{ 0.0 };    ///< 机械止挡阈值 N·m
    double homing_timeout_s{ 0.0 };           ///< homing 超时 s
};

/**
 * @brief 校验固定 EEF 位置参数
 * @param configured 配置值
 * @param expected 固定参考值
 * @return 两者有限且误差在容限内时返回 true
 */
bool matches_fixed_position(double configured, double expected) noexcept {
    return std::isfinite(configured) && std::abs(configured - expected) <= 1e-9;
}

/**
 * @brief 加载并验证 EEF 硬件配置
 * @param path 配置文件路径
 * @return 有效配置或 INVALID_CONFIG
 */
tl::expected<DriverConfig, DamiaoEefError> load_config(const std::string& path) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node damiao_node = root["damiao"];
        const YAML::Node motor_node = damiao_node["motor"];
        const YAML::Node eef_node = root["eef"];
        const YAML::Node homing_node = eef_node["homing"];

        DriverConfig config;
        config.bus = damiao_node["bus"].as<std::string>();
        config.serial_port = damiao_node["serial_port"].as<std::string>();
        config.baudrate = damiao_node["baudrate"].as<int>();
        config.motor_id = motor_node["motor_id"].as<std::uint32_t>();
        config.master_id = motor_node["master_id"].as<std::uint32_t>();
        const std::string motor_type = motor_node["motor_type"].as<std::string>();
        const double open_position = eef_node["open_position"].as<double>();
        const double closed_position = eef_node["closed_position"].as<double>();
        config.max_gripper_torque = eef_node["max_gripper_torque"].as<double>();
        config.homing_velocity = homing_node["velocity"].as<double>();
        config.homing_torque_threshold = homing_node["torque_threshold"].as<double>();
        config.homing_timeout_s = homing_node["timeout_s"].as<double>();

        const bool valid = !config.bus.empty() && !config.serial_port.empty() && config.baudrate > 0 &&
            config.motor_id <= kCanIdMaximum && config.master_id <= kCanIdMaximum &&
            config.motor_id == kEefMotorId && config.master_id == kEefMasterId &&
            motor_type == "DM4310" &&
            matches_fixed_position(open_position, kEefMotorOpenPosition) &&
            matches_fixed_position(closed_position, kEefMotorClosedPosition) &&
            std::isfinite(config.homing_velocity) && config.homing_velocity > 0.0 &&
            std::isfinite(config.homing_torque_threshold) && config.homing_torque_threshold > 0.0 &&
            std::isfinite(config.homing_timeout_s) && config.homing_timeout_s > 0.0 &&
            calculate_velocity_limit().has_value() &&
            calculate_current_limit(config.max_gripper_torque).has_value();
        if(!valid) return tl::make_unexpected(DamiaoEefError::INVALID_CONFIG);
        return config;
    }
    catch(const std::exception&) {
        return tl::make_unexpected(DamiaoEefError::INVALID_CONFIG);
    }
}

/**
 * @brief 将共享通道错误映射到 EEF Driver 错误
 * @param error 通道错误
 * @return 对应的 EEF Driver 错误
 */
DamiaoEefError map_channel_error(serial_arm::protocol::damiao_usb2can::Err error) noexcept {
    using ChannelError = serial_arm::protocol::damiao_usb2can::Err;
    switch(error) {
        case ChannelError::OPEN_FAILED:
            return DamiaoEefError::OPEN_FAILED;
        case ChannelError::CONFIG_CONFLICT:
            return DamiaoEefError::BUS_CONFIG_CONFLICT;
        case ChannelError::TYPE_MISMATCH:
            return DamiaoEefError::BUS_TYPE_MISMATCH;
    }
    return DamiaoEefError::OPEN_FAILED;
}

} // namespace

/**
 * @brief 真实 Driver 的共享通道与达妙对象所有权
 */
struct DamiaoEefDriver::Impl {
    DriverConfig config;                                      ///< 已验证硬件配置
    std::shared_ptr<serial_arm::transport::CanChannel> channel;  ///< main_can 共享通道
    std::unique_ptr<damiao::Motor> motor;                      ///< DM4310 电机对象
    std::unique_ptr<damiao::MotorControl> control;             ///< 达妙控制对象
    bool configured{ false };                                 ///< 配置是否完成
    bool active{ false };                                     ///< Torque_Pos 是否可用
};

// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 主动刷新并校验一帧 EEF 电机反馈
 * @param impl Driver 内部对象
 * @return 有效反馈或 FEEDBACK_FAILED
 */
tl::expected<EefMotorFeedback, DamiaoEefError> refresh_impl(DamiaoEefDriver::Impl& impl) {
    if(!impl.control || !impl.motor || !impl.control->refresh_motor_status(*impl.motor)) {
        return tl::make_unexpected(DamiaoEefError::FEEDBACK_FAILED);
    }
    EefMotorFeedback feedback{
        static_cast<double>(impl.motor->get_position()),
        static_cast<double>(impl.motor->get_velocity()),
        static_cast<double>(impl.motor->get_tau()),
    };
    if(!std::isfinite(feedback.position) || !std::isfinite(feedback.velocity) ||
        !std::isfinite(feedback.torque)) {
        return tl::make_unexpected(DamiaoEefError::FEEDBACK_FAILED);
    }
    return feedback;
}

/**
 * @brief 发送零速度并安全失能
 * @param impl Driver 内部对象
 * @param velocity_mode 当前是否处于速度模式
 */
void stop_velocity_and_disable(DamiaoEefDriver::Impl& impl, bool velocity_mode) noexcept {
    if(!impl.control || !impl.motor) return;
    try {
        if(velocity_mode) impl.control->control_vel(*impl.motor, 0.0F);
        (void)impl.control->disable(*impl.motor);
    }
    catch(...) {
    }
    impl.active = false;
}

/**
 * @brief 在 lifecycle 取消时执行安全停止
 * @param impl Driver 内部对象
 * @param cancel_requested 原子取消标志
 * @param velocity_mode 当前是否处于速度模式
 * @return 未取消时成功；已取消时返回 CANCELLED
 */
tl::expected<void, DamiaoEefError> cancel_activation_if_requested(
    DamiaoEefDriver::Impl& impl,
    const std::atomic_bool& cancel_requested,
    bool velocity_mode) noexcept {
    if(!cancel_requested.load()) return {};
    stop_velocity_and_disable(impl, velocity_mode);
    return tl::make_unexpected(DamiaoEefError::CANCELLED);
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 把 Driver 错误转换为稳定日志文本
 * @param error Driver 错误
 * @return 静态错误文本
 */
const char* damiao_eef_error_string(DamiaoEefError error) noexcept {
    switch(error) {
        case DamiaoEefError::NOT_CONFIGURED: return "not configured";
        case DamiaoEefError::NOT_ACTIVE: return "not active";
        case DamiaoEefError::INVALID_CONFIG: return "invalid configuration";
        case DamiaoEefError::OPEN_FAILED: return "shared CAN channel open failed";
        case DamiaoEefError::BUS_CONFIG_CONFLICT: return "main_can configuration conflict";
        case DamiaoEefError::BUS_TYPE_MISMATCH: return "main_can bus type mismatch";
        case DamiaoEefError::ENABLE_FAILED: return "motor enable failed";
        case DamiaoEefError::DISABLE_FAILED: return "motor disable failed";
        case DamiaoEefError::KP_APR_FAILED: return "KP_APR write verification failed";
        case DamiaoEefError::MODE_SWITCH_FAILED: return "control mode switch failed";
        case DamiaoEefError::FEEDBACK_FAILED: return "motor feedback refresh failed";
        case DamiaoEefError::HOMING_TIMEOUT: return "homing timeout";
        case DamiaoEefError::CANCELLED: return "activation cancelled";
        case DamiaoEefError::COMMAND_FAILED: return "motor command failed";
    }
    return "unknown error";
}

/**
 * @brief 构造未配置的 DM4310 EEF Driver
 */
DamiaoEefDriver::DamiaoEefDriver()
    : impl_(std::make_unique<Impl>()) {
}

/**
 * @brief 安全释放 EEF Driver 资源
 */
DamiaoEefDriver::~DamiaoEefDriver() {
    cleanup();
}

/**
 * @brief 读取配置并获取 main_can 共享通道
 * @param config_path damiao_eef.yaml 路径
 * @return 配置与通道创建结果
 */
tl::expected<void, DamiaoEefError> DamiaoEefDriver::configure(const std::string& config_path) {
    cleanup();
    auto config = load_config(config_path);
    if(!config) return tl::make_unexpected(config.error());

    serial_arm::protocol::damiao_usb2can::Config bus_config;
    bus_config.serial_port = config->serial_port;
    bus_config.baudrate = config->baudrate;
    std::vector<serial_arm::transport::CanFilter> filters{
        serial_arm::transport::CanFilter{ config->motor_id, kCanFilterMask },
        serial_arm::transport::CanFilter{ config->master_id, kCanFilterMask },
    };
    auto channel = serial_arm::protocol::damiao_usb2can::acquire_channel(
        config->bus, bus_config, std::move(filters));
    if(!channel) return tl::make_unexpected(map_channel_error(channel.error()));

    try {
        impl_->config = *config;
        impl_->channel = *channel;
        impl_->motor = std::make_unique<damiao::Motor>(
            damiao::DM4310, impl_->config.motor_id, impl_->config.master_id);
        impl_->control = std::make_unique<damiao::MotorControl>(impl_->channel);
        impl_->control->add_motor(impl_->motor.get());
        impl_->configured = true;
        return {};
    }
    catch(const std::exception&) {
        cleanup();
        return tl::make_unexpected(DamiaoEefError::INVALID_CONFIG);
    }
}

/**
 * @brief 使能并执行 open-stop homing，最后切换 Torque_Pos
 * @param cancel_requested lifecycle deactivate/shutdown 取消标志
 * @return homing、取消与模式切换结果
 */
tl::expected<void, DamiaoEefError> DamiaoEefDriver::activate(
    const std::atomic_bool& cancel_requested) {
    if(!impl_->configured || !impl_->control || !impl_->motor) {
        return tl::make_unexpected(DamiaoEefError::NOT_CONFIGURED);
    }
    if(impl_->active) return {};

    bool velocity_mode = false;
    try {
        auto cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        if(!impl_->control->enable(*impl_->motor)) {
            stop_velocity_and_disable(*impl_, velocity_mode);
            return tl::make_unexpected(DamiaoEefError::ENABLE_FAILED);
        }
        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        if(!impl_->control->change_motor_param(*impl_->motor, damiao::KP_APR, static_cast<float>(kKpApr))) {
            stop_velocity_and_disable(*impl_, velocity_mode);
            return tl::make_unexpected(DamiaoEefError::KP_APR_FAILED);
        }
        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        if(!impl_->control->switch_control_mode(*impl_->motor, damiao::VEL_MODE)) {
            stop_velocity_and_disable(*impl_, velocity_mode);
            return tl::make_unexpected(DamiaoEefError::MODE_SWITCH_FAILED);
        }
        velocity_mode = true;
        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        impl_->control->control_vel(*impl_->motor, static_cast<float>(impl_->config.homing_velocity));
        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        EefHomingStateMachine homing(
            impl_->config.homing_torque_threshold,
            std::chrono::duration<double>(impl_->config.homing_timeout_s));
        homing.start(EefHomingStateMachine::Clock::now());
        while(true) {
            cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
            if(!cancelled) return cancelled;

            const auto feedback = refresh_impl(*impl_);
            cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
            if(!cancelled) return cancelled;

            const auto decision = homing.observe(
                feedback ? std::optional<double>{ feedback->torque } : std::nullopt,
                EefHomingStateMachine::Clock::now());
            if(decision == HomingDecision::CONTINUE) {
                cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
                if(!cancelled) return cancelled;
                std::this_thread::sleep_for(kHomingPollPeriod);
                cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
                if(!cancelled) return cancelled;
                continue;
            }
            if(decision == HomingDecision::FEEDBACK_FAILURE) {
                stop_velocity_and_disable(*impl_, velocity_mode);
                return tl::make_unexpected(DamiaoEefError::FEEDBACK_FAILED);
            }
            if(decision == HomingDecision::TIMEOUT) {
                stop_velocity_and_disable(*impl_, velocity_mode);
                return tl::make_unexpected(DamiaoEefError::HOMING_TIMEOUT);
            }
            break;
        }

        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        impl_->control->control_vel(*impl_->motor, 0.0F);
        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        if(!impl_->control->disable(*impl_->motor)) {
            stop_velocity_and_disable(*impl_, velocity_mode);
            return tl::make_unexpected(DamiaoEefError::DISABLE_FAILED);
        }
        velocity_mode = false;
        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        impl_->control->set_zero_position(*impl_->motor);
        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        if(!impl_->control->enable(*impl_->motor)) {
            stop_velocity_and_disable(*impl_, false);
            return tl::make_unexpected(DamiaoEefError::ENABLE_FAILED);
        }
        velocity_mode = true;
        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        if(!impl_->control->switch_control_mode(*impl_->motor, damiao::POS_FORCE_MODE)) {
            stop_velocity_and_disable(*impl_, false);
            return tl::make_unexpected(DamiaoEefError::MODE_SWITCH_FAILED);
        }
        velocity_mode = false;
        cancelled = cancel_activation_if_requested(*impl_, cancel_requested, velocity_mode);
        if(!cancelled) return cancelled;

        impl_->active = true;
        return {};
    }
    catch(const std::exception&) {
        stop_velocity_and_disable(*impl_, velocity_mode);
        if(cancel_requested.load()) return tl::make_unexpected(DamiaoEefError::CANCELLED);
        return tl::make_unexpected(DamiaoEefError::COMMAND_FAILED);
    }
}

/**
 * @brief 主动刷新电机 position、velocity 与 torque
 * @return 最新反馈或 Driver 错误
 */
tl::expected<EefMotorFeedback, DamiaoEefError> DamiaoEefDriver::refresh() {
    if(!impl_->configured) return tl::make_unexpected(DamiaoEefError::NOT_CONFIGURED);
    if(!impl_->active) return tl::make_unexpected(DamiaoEefError::NOT_ACTIVE);
    try {
        return refresh_impl(*impl_);
    }
    catch(const std::exception&) {
        return tl::make_unexpected(DamiaoEefError::FEEDBACK_FAILED);
    }
}

/**
 * @brief 发送归一化位置 Torque_Pos 命令
 * @param position_command 0 为 open、1 为 closed
 * @return 命令发送结果
 */
tl::expected<void, DamiaoEefError> DamiaoEefDriver::command_position(double position_command) {
    if(!impl_->configured) return tl::make_unexpected(DamiaoEefError::NOT_CONFIGURED);
    if(!impl_->active) return tl::make_unexpected(DamiaoEefError::NOT_ACTIVE);
    const auto command = make_torque_pos_command(position_command, impl_->config.max_gripper_torque);
    if(!command) return tl::make_unexpected(DamiaoEefError::INVALID_CONFIG);
    try {
        impl_->control->control_pos_force(
            *impl_->motor, command->target_position, command->velocity_limit, command->current_limit);
        return {};
    }
    catch(const std::exception&) {
        return tl::make_unexpected(DamiaoEefError::COMMAND_FAILED);
    }
}

/**
 * @brief 刷新当前位置并以 Torque_Pos 保持
 * @return hold 命令结果；反馈无效时安全失能
 */
tl::expected<void, DamiaoEefError> DamiaoEefDriver::stop() {
    if(!impl_->configured) return tl::make_unexpected(DamiaoEefError::NOT_CONFIGURED);
    if(!impl_->active) return tl::make_unexpected(DamiaoEefError::NOT_ACTIVE);
    try {
        const auto feedback = refresh_impl(*impl_);
        if(!feedback) {
            stop_velocity_and_disable(*impl_, false);
            return tl::make_unexpected(feedback.error());
        }
        const auto velocity = calculate_velocity_limit();
        const auto current = calculate_current_limit(impl_->config.max_gripper_torque);
        if(!velocity || !current) {
            stop_velocity_and_disable(*impl_, false);
            return tl::make_unexpected(DamiaoEefError::INVALID_CONFIG);
        }
        impl_->control->control_pos_force(
            *impl_->motor, static_cast<float>(feedback->position), *velocity, *current);
        return {};
    }
    catch(const std::exception&) {
        stop_velocity_and_disable(*impl_, false);
        return tl::make_unexpected(DamiaoEefError::COMMAND_FAILED);
    }
}

/**
 * @brief 保持当前位置后失能电机
 * @return 停止与失能结果
 */
tl::expected<void, DamiaoEefError> DamiaoEefDriver::deactivate() {
    if(!impl_->configured) return tl::make_unexpected(DamiaoEefError::NOT_CONFIGURED);
    if(!impl_->active) return {};

    const auto stopped = stop();
    if(!impl_->active) return stopped;
    try {
        const bool disabled = impl_->control->disable(*impl_->motor);
        impl_->active = false;
        if(!disabled) return tl::make_unexpected(DamiaoEefError::DISABLE_FAILED);
        return stopped;
    }
    catch(const std::exception&) {
        impl_->active = false;
        return tl::make_unexpected(DamiaoEefError::DISABLE_FAILED);
    }
}

/**
 * @brief 释放 EEF 通道与底层 Driver
 */
void DamiaoEefDriver::cleanup() noexcept {
    if(!impl_) return;
    if(impl_->active) {
        try {
            (void)deactivate();
        }
        catch(...) {
        }
    }
    impl_->active = false;
    impl_->configured = false;
    impl_->control.reset();
    impl_->motor.reset();
    impl_->channel.reset();
    impl_->config = DriverConfig{};
}

/**
 * @brief 查询是否已完成 homing 且处于 Torque_Pos
 * @return active 状态
 */
bool DamiaoEefDriver::is_active() const noexcept {
    return impl_ && impl_->active;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace tomato_picker_eef
