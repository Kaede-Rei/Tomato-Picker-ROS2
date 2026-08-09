#include "tomato_picker_eef/eef_worker.hpp"

#include "rclcpp/logging.hpp"
#include "tomato_picker_eef/eef_model.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <exception>
#include <utility>

namespace tomato_picker_eef {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

constexpr double kDefaultIoRateHz = 20.0;                    ///< 默认反馈频率 Hz
constexpr std::size_t kDefaultMaxFeedbackFailures = 3;       ///< 默认连续反馈失败门限

/**
 * @brief Worker 配置文件中的非实时周期参数
 */
struct WorkerConfig {
    std::chrono::steady_clock::duration io_period;                        ///< 反馈周期
    std::size_t max_feedback_failures{ kDefaultMaxFeedbackFailures };     ///< 连续反馈失败门限
};

/**
 * @brief 读取 Worker 周期与反馈失败门限
 * @param path EEF 配置文件路径
 * @return 成功时返回 WorkerConfig；失败时返回 INVALID_CONFIG
 */
tl::expected<WorkerConfig, DamiaoEefError> load_worker_config(const std::string& path) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node eef = root["eef"];
        const double io_rate_hz = eef["io_rate_hz"].as<double>(kDefaultIoRateHz);
        const int max_failures = eef["max_consecutive_feedback_failures"].as<int>(
            static_cast<int>(kDefaultMaxFeedbackFailures));
        if(!std::isfinite(io_rate_hz) || io_rate_hz <= 0.0 || max_failures <= 0) {
            return tl::make_unexpected(DamiaoEefError::INVALID_CONFIG);
        }

        const auto period = std::chrono::duration<double>(1.0 / io_rate_hz);
        const auto steady_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        if(steady_period <= std::chrono::steady_clock::duration::zero()) {
            return tl::make_unexpected(DamiaoEefError::INVALID_CONFIG);
        }
        return WorkerConfig{ steady_period, static_cast<std::size_t>(max_failures) };
    }
    catch(const std::exception&) {
        return tl::make_unexpected(DamiaoEefError::INVALID_CONFIG);
    }
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 使用真实 Damiao Driver 构造 Worker
 * @param logger Worker 日志器
 */
EefWorker::EefWorker(rclcpp::Logger logger)
    : EefWorker(std::move(logger), std::make_unique<DamiaoEefDriver>()) {
}

/**
 * @brief 使用指定 Driver 构造 Worker
 * @param logger Worker 日志器
 * @param driver Worker 独占的 Driver
 */
EefWorker::EefWorker(rclcpp::Logger logger, std::unique_ptr<EefIoDriver> driver)
    : logger_(std::move(logger)), driver_(std::move(driver)) {
}

/**
 * @brief 析构 Worker 并等待 I/O 线程退出
 */
EefWorker::~EefWorker() {
    shutdown();
}

/**
 * @brief 配置 Driver 和 Worker 周期参数
 * @param config_path EEF 配置文件路径
 * @return 成功时返回空值；失败时返回 DamiaoEefError
 */
tl::expected<void, DamiaoEefError> EefWorker::configure(const std::string& config_path) {
    if(!driver_ || thread_.joinable()) {
        return tl::make_unexpected(DamiaoEefError::INVALID_CONFIG);
    }
    auto worker_config = load_worker_config(config_path);
    if(!worker_config) return tl::make_unexpected(worker_config.error());

    auto configured = driver_->configure(config_path);
    if(!configured) return configured;
    io_period_ = worker_config->io_period;
    max_feedback_failures_ = worker_config->max_feedback_failures;
    consecutive_feedback_failures_ = 0;
    configured_ = true;
    reset_command_state();
    set_state(EefWorkerState::INACTIVE);
    return {};
}

/**
 * @brief 启动非实时 I/O 线程
 * @return 线程启动成功时返回 true
 */
bool EefWorker::start() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!configured_ || !driver_ || thread_.joinable() || shutdown_requested_) return false;
    try {
        thread_ = std::thread(&EefWorker::run, this);
        return true;
    }
    catch(const std::exception& exception) {
        RCLCPP_ERROR(logger_, "Unable to start EEF worker: %s", exception.what());
        return false;
    }
}

/**
 * @brief 非阻塞请求 homing 和 activate
 */
void EefWorker::request_activate() noexcept {
    cancel_requested_.store(false);
    set_active(false);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(shutdown_requested_) return;
        deactivate_requested_ = false;
        activate_requested_ = true;
        set_state(EefWorkerState::HOMING);
    }
    cv_.notify_all();
}

/**
 * @brief 非阻塞请求 stop 和 deactivate
 */
void EefWorker::request_deactivate() noexcept {
    cancel_requested_.store(true);
    set_active(false);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(shutdown_requested_) return;
        activate_requested_ = false;
        deactivate_requested_ = true;
    }
    cv_.notify_all();
}

/**
 * @brief 提交 EEF 业务命令并唤醒 Worker
 * @param command EEF 命令
 * @param value SET_POSITION 归一化位置
 * @return 命令提交结果
 */
SubmitResult EefWorker::submit(EefCommand command, double value) noexcept {
    if(!active_.load()) return SubmitResult::NOT_ACTIVE;

    PendingEefCommand pending;
    switch(command) {
        case EefCommand::OPEN:
            pending = PendingEefCommand{ command, kEefCommandMinimum };
            break;
        case EefCommand::CLOSE:
            pending = PendingEefCommand{ command, kEefCommandMaximum };
            break;
        case EefCommand::STOP:
            pending = PendingEefCommand{ command, 0.0 };
            break;
        case EefCommand::SET_POSITION:
            if(!std::isfinite(value) || value < kEefCommandMinimum || value > kEefCommandMaximum) {
                return SubmitResult::OUT_OF_RANGE;
            }
            pending = PendingEefCommand{ command, value };
            break;
        default:
            return SubmitResult::INVALID_COMMAND;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!active_.load() || shutdown_requested_) return SubmitResult::NOT_ACTIVE;
        if(pending_ && pending_->command == EefCommand::STOP && command != EefCommand::STOP) {
            return SubmitResult::BUSY;
        }
        pending_ = pending;
        command_requested_ = true;
    }
    cv_.notify_all();
    return SubmitResult::ACCEPTED;
}

/**
 * @brief 请求退出并等待 I/O 线程结束
 */
void EefWorker::shutdown() noexcept {
    cancel_requested_.store(true);
    set_active(false);
    bool joinable = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_requested_ = true;
        activate_requested_ = false;
        deactivate_requested_ = false;
        command_requested_ = false;
        joinable = thread_.joinable();
    }
    cv_.notify_all();

    if(joinable) {
        try {
            thread_.join();
        }
        catch(const std::exception& exception) {
            RCLCPP_ERROR(logger_, "EEF worker join failed: %s", exception.what());
        }
    }
    else if(driver_ && configured_) {
        driver_->cleanup();
        configured_ = false;
        hardware_ready_.store(false);
        reset_command_state();
        set_state(EefWorkerState::SHUTDOWN);
    }
}

/**
 * @brief 获取 Worker 当前状态
 * @return 原子状态快照
 */
EefWorkerState EefWorker::state() const noexcept {
    return state_.load();
}

/**
 * @brief 查询硬件是否完成 homing 并处于 READY
 * @return READY 时返回 true
 */
bool EefWorker::hardware_ready() const noexcept {
    return hardware_ready_.load();
}

/**
 * @brief 查询 Worker 是否接受业务命令
 * @return READY 且可接受命令时返回 true
 */
bool EefWorker::is_active() const noexcept {
    return active_.load();
}

/**
 * @brief 查询最新归一化 EEF 反馈
 * @return 已有有效反馈时返回位置
 */
std::optional<double> EefWorker::last_position() const noexcept {
    if(!has_feedback_.load()) return std::nullopt;
    return last_position_.load();
}

/**
 * @brief 等待指定状态
 * @param expected 目标状态
 * @param timeout 最大等待时间
 * @return 在 timeout 前到达目标状态时返回 true
 */
bool EefWorker::wait_for_state(EefWorkerState expected, std::chrono::milliseconds timeout) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [this, expected]() { return state_.load() == expected; });
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 运行 Worker 生命周期、命令与反馈循环
 */
void EefWorker::run() noexcept {
    using Clock = std::chrono::steady_clock;
    auto next_refresh = Clock::time_point::max();

    while(true) {
        bool shutdown = false;
        bool deactivate = false;
        bool activate = false;
        bool command = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            const auto requested = [this]() {
                return shutdown_requested_ || deactivate_requested_ ||
                    activate_requested_ || command_requested_;
            };
            if(state_.load() == EefWorkerState::READY) {
                cv_.wait_until(lock, next_refresh, requested);
            }
            else {
                cv_.wait(lock, requested);
            }

            shutdown = shutdown_requested_;
            deactivate = deactivate_requested_;
            activate = activate_requested_;
            command = command_requested_;
            if(deactivate) deactivate_requested_ = false;
            if(activate) activate_requested_ = false;
            if(command) command_requested_ = false;
        }

        if(shutdown) break;
        if(deactivate) {
            handle_deactivate();
            next_refresh = Clock::time_point::max();
            continue;
        }
        if(activate) {
            handle_activate();
            next_refresh = state_.load() == EefWorkerState::READY ? Clock::now() : Clock::time_point::max();
            continue;
        }
        if(state_.load() != EefWorkerState::READY) continue;

        if(command && !handle_command()) {
            next_refresh = Clock::time_point::max();
            continue;
        }
        if(command || Clock::now() >= next_refresh) {
            if(!refresh_feedback()) {
                next_refresh = Clock::time_point::max();
                continue;
            }
            next_refresh = Clock::now() + io_period_;
        }
    }

    reset_command_state();
    if(driver_ && driver_->is_active()) {
        const auto deactivated = driver_->deactivate();
        if(!deactivated) {
            RCLCPP_ERROR(logger_, "EEF shutdown deactivate failed: %s",
                damiao_eef_error_string(deactivated.error()));
        }
    }
    if(driver_) driver_->cleanup();
    configured_ = false;
    hardware_ready_.store(false);
    set_state(EefWorkerState::SHUTDOWN);
}

/**
 * @brief 执行 activate 请求
 */
void EefWorker::handle_activate() noexcept {
    set_active(false);
    hardware_ready_.store(false);
    consecutive_feedback_failures_ = 0;
    set_state(EefWorkerState::HOMING);
    RCLCPP_INFO(logger_, "EEF worker starting homing");

    const auto activated = driver_->activate(cancel_requested_);
    if(!activated) {
        const bool lifecycle_cancellation = activated.error() == DamiaoEefError::CANCELLED &&
            cancel_requested_.load();
        if(lifecycle_cancellation) {
            set_active(false);
            hardware_ready_.store(false);
            RCLCPP_INFO(logger_, "EEF homing cancelled by lifecycle request");
            return;
        }
        set_state(EefWorkerState::ERROR);
        RCLCPP_ERROR(logger_, "EEF worker homing/activate failed: %s",
            damiao_eef_error_string(activated.error()));
        return;
    }

    bool lifecycle_stop_pending = cancel_requested_.load();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lifecycle_stop_pending = lifecycle_stop_pending ||
            deactivate_requested_ || shutdown_requested_;
    }
    if(lifecycle_stop_pending) return;

    hardware_ready_.store(true);
    set_active(true);
    set_state(EefWorkerState::READY);
    RCLCPP_INFO(logger_, "EEF homing complete; worker READY at configured rate");
}

/**
 * @brief 执行 deactivate 请求
 */
void EefWorker::handle_deactivate() noexcept {
    set_active(false);
    hardware_ready_.store(false);
    consecutive_feedback_failures_ = 0;
    const auto deactivated = driver_->deactivate();
    if(!deactivated) {
        RCLCPP_ERROR(logger_, "EEF worker deactivate failed: %s",
            damiao_eef_error_string(deactivated.error()));
    }
    set_state(EefWorkerState::INACTIVE);
    RCLCPP_INFO(logger_, "EEF worker INACTIVE");
}

/**
 * @brief 执行一个 pending command
 * @return 成功或没有 pending command 时返回 true
 */
bool EefWorker::handle_command() noexcept {
    const auto pending = take_pending();
    if(!pending) return true;
    const auto result = pending->command == EefCommand::STOP ?
        driver_->stop() : driver_->command_position(pending->value);
    if(result) return true;
    enter_error("command", result.error());
    return false;
}

/**
 * @brief 刷新一次反馈并执行连续失败门限
 * @return 硬件仍可继续运行时返回 true
 */
bool EefWorker::refresh_feedback() noexcept {
    const auto feedback = driver_->refresh();
    if(feedback) {
        consecutive_feedback_failures_ = 0;
        update_feedback(motor_position_to_command(feedback->position));
        return true;
    }

    ++consecutive_feedback_failures_;
    RCLCPP_WARN(logger_, "EEF feedback refresh failed (%zu/%zu): %s",
        consecutive_feedback_failures_, max_feedback_failures_,
        damiao_eef_error_string(feedback.error()));
    if(consecutive_feedback_failures_ < max_feedback_failures_) return true;

    enter_error("consecutive feedback", feedback.error());
    return false;
}

/**
 * @brief 执行错误后的安全失能
 * @param context 错误日志上下文
 * @param error 首个 Driver 错误
 */
void EefWorker::enter_error(const char* context, DamiaoEefError error) noexcept {
    set_active(false);
    hardware_ready_.store(false);
    RCLCPP_ERROR(logger_, "EEF worker %s failure; entering safe ERROR: %s",
        context, damiao_eef_error_string(error));
    if(driver_->is_active()) {
        const auto deactivated = driver_->deactivate();
        if(!deactivated) {
            RCLCPP_ERROR(logger_, "EEF safe deactivate failed: %s",
                damiao_eef_error_string(deactivated.error()));
        }
    }
    set_state(EefWorkerState::ERROR);
}

/**
 * @brief 切换命令接收状态并在失活时清空邮箱
 * @param active 是否接受业务命令
 */
void EefWorker::set_active(bool active) noexcept {
    active_.store(active);
    if(active) return;
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
    command_requested_ = false;
}

/**
 * @brief 取出并清空待执行命令
 * @return 没有 pending 命令时返回空
 */
std::optional<PendingEefCommand> EefWorker::take_pending() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    auto pending = pending_;
    pending_.reset();
    return pending;
}

/**
 * @brief 更新最新归一化反馈
 * @param position 归一化位置
 */
void EefWorker::update_feedback(double position) noexcept {
    if(!std::isfinite(position)) return;
    last_position_.store(position);
    has_feedback_.store(true);
}

/**
 * @brief 清空命令邮箱、激活和反馈状态
 */
void EefWorker::reset_command_state() noexcept {
    active_.store(false);
    has_feedback_.store(false);
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
    command_requested_ = false;
}

/**
 * @brief 原子更新状态并唤醒状态观察者
 * @param state 新状态
 */
void EefWorker::set_state(EefWorkerState state) noexcept {
    state_.store(state);
    cv_.notify_all();
}

} // namespace tomato_picker_eef
