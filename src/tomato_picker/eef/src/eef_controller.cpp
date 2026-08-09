#include "tomato_picker_eef/eef_controller.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace tomato_picker_eef {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

constexpr std::int32_t kSuccess = 0;          ///< 命令已接受
constexpr std::int32_t kInvalidCommand = 1;   ///< 命令类型非法
constexpr std::int32_t kOutOfRange = 2;       ///< 位置超出范围
constexpr std::int32_t kNotActive = 3;        ///< Worker 未激活
constexpr std::int32_t kBusy = 4;             ///< STOP 命令尚未消费
constexpr std::int32_t kHoming = 5;           ///< EEF 正在 homing

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 声明 controller 参数
 * @return lifecycle 结果
 */
EefController::CallbackReturn EefController::on_init() {
    try {
        auto_declare<std::string>("service_name", "/tomato_picker/eef/command");
        auto_declare<std::string>("eef_config", "");
        return CallbackReturn::SUCCESS;
    }
    catch(const std::exception& exception) {
        RCLCPP_ERROR(get_node()->get_logger(), "EEF parameter declaration failed: %s", exception.what());
        return CallbackReturn::ERROR;
    }
}

/**
 * @brief 返回空 command interface 配置
 * @return NONE interface 配置
 */
controller_interface::InterfaceConfiguration EefController::command_interface_configuration() const {
    return { controller_interface::interface_configuration_type::NONE, {} };
}

/**
 * @brief 返回空 state interface 配置
 * @return NONE interface 配置
 */
controller_interface::InterfaceConfiguration EefController::state_interface_configuration() const {
    return { controller_interface::interface_configuration_type::NONE, {} };
}

/**
 * @brief 配置共享 main_can、创建 Worker 与 Service
 * @param previous_state 前一 lifecycle 状态
 * @return lifecycle 结果
 */
EefController::CallbackReturn EefController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    static_cast<void>(previous_state);
    const auto node = get_node();
    service_name_ = node->get_parameter("service_name").as_string();
    eef_config_ = node->get_parameter("eef_config").as_string();
    if(eef_config_.empty()) {
        try {
            eef_config_ = ament_index_cpp::get_package_share_directory("tomato_picker_eef") +
                "/config/damiao_eef.yaml";
        }
        catch(const std::exception& exception) {
            RCLCPP_ERROR(node->get_logger(), "Unable to resolve damiao_eef.yaml: %s", exception.what());
            return CallbackReturn::ERROR;
        }
    }

    auto worker = std::make_shared<EefWorker>(node->get_logger());
    const auto configured = worker->configure(eef_config_);
    if(!configured) {
        RCLCPP_ERROR(node->get_logger(), "EEF Driver configure failed: %s; config=%s",
            damiao_eef_error_string(configured.error()), eef_config_.c_str());
        return CallbackReturn::ERROR;
    }
    if(!worker->start()) {
        RCLCPP_ERROR(node->get_logger(), "EEF worker start failed");
        return CallbackReturn::ERROR;
    }
    std::atomic_store(&worker_, std::move(worker));

    command_service_ = node->create_service<CommandEef>(
        service_name_,
        std::bind(&EefController::command_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(node->get_logger(), "EEF configured with non-realtime worker; service=%s config=%s",
        service_name_.c_str(), eef_config_.c_str());
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 非阻塞提交 EEF homing 和 activate 请求
 * @param previous_state 前一 lifecycle 状态
 * @return lifecycle 结果
 */
EefController::CallbackReturn EefController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    static_cast<void>(previous_state);
    const auto worker = std::atomic_load(&worker_);
    if(!worker) {
        RCLCPP_ERROR(get_node()->get_logger(), "EEF worker is not configured");
        return CallbackReturn::ERROR;
    }
    worker->request_activate();
    RCLCPP_INFO(get_node()->get_logger(), "EEF homing requested asynchronously");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 执行 ros2_control realtime 空操作
 * @param time 当前时间
 * @param period 控制周期
 * @return OK
 */
controller_interface::return_type EefController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
    static_cast<void>(time);
    static_cast<void>(period);
    return controller_interface::return_type::OK;
}

/**
 * @brief 非阻塞提交 stop 和 deactivate 请求
 * @param previous_state 前一 lifecycle 状态
 * @return lifecycle 结果
 */
EefController::CallbackReturn EefController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    static_cast<void>(previous_state);
    const auto worker = std::atomic_load(&worker_);
    if(worker) worker->request_deactivate();
    RCLCPP_INFO(get_node()->get_logger(), "EEF stop/deactivate requested asynchronously");
    return CallbackReturn::SUCCESS;
}

/**
 * @brief 停止 Worker 并释放 ROS Service
 * @param previous_state 前一 lifecycle 状态
 * @return lifecycle 结果
 */
EefController::CallbackReturn EefController::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    static_cast<void>(previous_state);
    command_service_.reset();
    auto worker = std::atomic_exchange(&worker_, std::shared_ptr<EefWorker>{});
    if(worker) worker->shutdown();
    return CallbackReturn::SUCCESS;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

/**
 * @brief 验证并提交 CommandEef 请求
 * @param request Service 请求
 * @param response Service 响应
 */
void EefController::command_callback(
    const CommandEef::Request::SharedPtr request,
    CommandEef::Response::SharedPtr response) {
    const auto command = static_cast<EefCommand>(request->command);
    const auto worker = std::atomic_load(&worker_);
    const bool position_command = command == EefCommand::OPEN || command == EefCommand::CLOSE ||
        command == EefCommand::SET_POSITION;
    if(worker && worker->state() == EefWorkerState::HOMING && position_command) {
        response->success = false;
        response->error_code = kHoming;
        response->message = "EEF is homing";
        return;
    }

    const auto result = worker ? worker->submit(command, request->value) : SubmitResult::NOT_ACTIVE;
    switch(result) {
        case SubmitResult::ACCEPTED:
            response->success = true;
            response->error_code = kSuccess;
            response->message = "eef command accepted";
            return;
        case SubmitResult::INVALID_COMMAND:
            response->error_code = kInvalidCommand;
            response->message = "unsupported eef command";
            break;
        case SubmitResult::OUT_OF_RANGE:
            response->error_code = kOutOfRange;
            response->message = "eef position must be within [0, 1]";
            break;
        case SubmitResult::NOT_ACTIVE:
            response->error_code = kNotActive;
            response->message = "eef controller is not active";
            break;
        case SubmitResult::BUSY:
            response->error_code = kBusy;
            response->message = "eef stop command is pending";
            break;
    }
    response->success = false;
}

} // namespace tomato_picker_eef

PLUGINLIB_EXPORT_CLASS(tomato_picker_eef::EefController, controller_interface::ControllerInterface)
