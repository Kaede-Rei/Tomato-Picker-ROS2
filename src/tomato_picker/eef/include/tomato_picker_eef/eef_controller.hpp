#pragma once

#include "controller_interface/controller_interface.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tomato_picker_eef/eef_worker.hpp"
#include "tomato_picker_interfaces/srv/command_eef.hpp"

#include <memory>
#include <string>

namespace tomato_picker_eef {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 与 SerialArm ros2_control hardware 同进程的达妙 EEF Controller
 */
class EefController final : public controller_interface::ControllerInterface {
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using CommandEef = tomato_picker_interfaces::srv::CommandEef;
    using Trigger = std_srvs::srv::Trigger;

    /**
     * @brief 声明 controller 参数
     * @return lifecycle 结果
     */
    CallbackReturn on_init() override;

    /**
     * @brief EEF 不 claim 机械臂 command interface
     * @return NONE interface 配置
     */
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    /**
     * @brief EEF 不 claim 机械臂 state interface
     * @return NONE interface 配置
     */
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    /**
     * @brief 配置共享 main_can、创建非实时 I/O Worker 与 Service
     * @param previous_state 前一 lifecycle 状态
     * @return lifecycle 结果
     */
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 非阻塞提交 EEF homing/activate 请求
     * @param previous_state 前一 lifecycle 状态
     * @return lifecycle 结果
     */
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief ros2_control realtime 空操作，不访问 EEF 硬件
     * @param time 当前时间
     * @param period 控制周期
     * @return controller 更新结果
     */
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /**
     * @brief 非阻塞提交 stop/deactivate 请求
     * @param previous_state 前一 lifecycle 状态
     * @return lifecycle 结果
     */
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 停止并 join I/O Worker，释放 Driver 与 ROS Service
     * @param previous_state 前一 lifecycle 状态
     * @return lifecycle 结果
     */
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

private:
    /**
     * @brief 验证并缓存 CommandEef 请求
     * @param request Service 请求
     * @param response Service 响应
     */
    void command_callback(
        const CommandEef::Request::SharedPtr request,
        CommandEef::Response::SharedPtr response);

    /**
     * @brief 返回 EEF Worker 的启动就绪状态
     * @param request Trigger 空请求
     * @param response READY 时 success=true，其余状态写入 message
     */
    void ready_callback(
        const Trigger::Request::SharedPtr request,
        Trigger::Response::SharedPtr response);

private:
    std::shared_ptr<EefWorker> worker_;                       ///< 同进程非实时硬件 I/O Worker
    rclcpp::Service<CommandEef>::SharedPtr command_service_; ///< EEF 业务 Service
    rclcpp::Service<Trigger>::SharedPtr ready_service_;       ///< EEF READY 查询 Service
    std::string service_name_;                               ///< CommandEef Service 名称
    std::string ready_service_name_;                         ///< READY 查询 Service 名称
    std::string eef_config_;                                 ///< EEF Driver 配置路径
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

} // namespace tomato_picker_eef
