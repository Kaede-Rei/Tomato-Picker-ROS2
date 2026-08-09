#include <gtest/gtest.h>

#include "rclcpp/logging.hpp"
#include "tomato_picker_eef/eef_controller.hpp"
#include "tomato_picker_eef/eef_worker.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

namespace tomato_picker_eef {
namespace {

using namespace std::chrono_literals;

class FakeEefDriver final : public EefIoDriver {
public:
    tl::expected<void, DamiaoEefError> configure(const std::string& config_path) override {
        static_cast<void>(config_path);
        ++configure_count;
        return {};
    }

    tl::expected<void, DamiaoEefError> activate(
        const std::atomic_bool& cancel_requested) override {
        ++activate_count;
        std::unique_lock<std::mutex> lock(mutex_);
        activate_entered_ = true;
        cv_.notify_all();
        while(!release_activate_ && !cancel_requested.load()) {
            cv_.wait_for(lock, 1ms);
        }
        if(cancel_requested.load()) {
            cancellation_observed_.store(true);
            active_.store(false);
            cv_.notify_all();
            return tl::make_unexpected(DamiaoEefError::CANCELLED);
        }
        if(activate_failure_) return tl::make_unexpected(DamiaoEefError::HOMING_TIMEOUT);
        active_.store(true);
        return {};
    }

    tl::expected<EefMotorFeedback, DamiaoEefError> refresh() override {
        ++refresh_count;
        std::unique_lock<std::mutex> lock(mutex_);
        refresh_entered_ = true;
        cv_.notify_all();
        while(block_refresh_ && !release_refresh_) cv_.wait(lock);
        if(refresh_failure_.load()) return tl::make_unexpected(DamiaoEefError::FEEDBACK_FAILED);
        return EefMotorFeedback{ -2.615, 0.0, 0.25 };
    }

    tl::expected<void, DamiaoEefError> command_position(double position_command) override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_position_command_ = position_command;
            ++command_count;
        }
        cv_.notify_all();
        return {};
    }

    tl::expected<void, DamiaoEefError> stop() override {
        ++stop_count;
        cv_.notify_all();
        return {};
    }

    tl::expected<void, DamiaoEefError> deactivate() override {
        active_.store(false);
        ++deactivate_count;
        return {};
    }

    void cleanup() noexcept override {
        active_.store(false);
        ++cleanup_count;
    }

    bool is_active() const noexcept override {
        return active_.load();
    }

    bool wait_for_activate(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this]() { return activate_entered_; });
    }

    void release_activate(bool fail = false) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            activate_failure_ = fail;
            release_activate_ = true;
        }
        cv_.notify_all();
    }

    bool wait_for_command_count(int expected, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this, expected]() { return command_count.load() >= expected; });
    }

    bool wait_for_stop_count(int expected, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this, expected]() { return stop_count.load() >= expected; });
    }

    bool wait_for_refresh_count(int expected, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this, expected]() { return refresh_count.load() >= expected; });
    }

    bool wait_for_refresh(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this]() { return refresh_entered_; });
    }

    bool wait_for_cancellation(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [this]() { return cancellation_observed_.load(); });
    }

    void block_refresh() {
        std::lock_guard<std::mutex> lock(mutex_);
        block_refresh_ = true;
        release_refresh_ = false;
        refresh_entered_ = false;
    }

    void release_refresh() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            release_refresh_ = true;
        }
        cv_.notify_all();
    }

    double last_position_command() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_position_command_;
    }

    void fail_refresh(bool fail) noexcept {
        refresh_failure_.store(fail);
    }

    std::atomic_int configure_count{ 0 };
    std::atomic_int activate_count{ 0 };
    std::atomic_int refresh_count{ 0 };
    std::atomic_int command_count{ 0 };
    std::atomic_int stop_count{ 0 };
    std::atomic_int deactivate_count{ 0 };
    std::atomic_int cleanup_count{ 0 };

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic_bool active_{ false };
    std::atomic_bool refresh_failure_{ false };
    std::atomic_bool cancellation_observed_{ false };
    bool activate_entered_{ false };
    bool release_activate_{ false };
    bool activate_failure_{ false };
    bool block_refresh_{ false };
    bool release_refresh_{ false };
    bool refresh_entered_{ false };
    double last_position_command_{ 0.0 };
};

struct WorkerFixture {
    FakeEefDriver* fake{ nullptr };
    EefWorker worker;

    WorkerFixture()
        : worker(rclcpp::get_logger("test_eef_worker"), make_driver()) {
        EXPECT_TRUE(worker.configure(EEF_TEST_CONFIG_PATH));
        EXPECT_TRUE(worker.start());
    }

    void activate_ready() {
        worker.request_activate();
        ASSERT_TRUE(fake->wait_for_activate(1s));
        fake->release_activate();
        ASSERT_TRUE(worker.wait_for_state(EefWorkerState::READY, 1s));
    }

private:
    std::unique_ptr<EefIoDriver> make_driver() {
        auto driver = std::make_unique<FakeEefDriver>();
        fake = driver.get();
        return driver;
    }
};

TEST(EefWorkerMailbox, RejectsCommandsWhileInactive) {
    WorkerFixture fixture;
    EXPECT_EQ(fixture.worker.submit(EefCommand::OPEN), SubmitResult::NOT_ACTIVE);
    EXPECT_EQ(fixture.fake->command_count.load(), 0);
}

TEST(EefWorkerMailbox, ProducesOpenCloseAndSetPositionTargets) {
    WorkerFixture fixture;
    fixture.activate_ready();

    EXPECT_EQ(fixture.worker.submit(EefCommand::OPEN), SubmitResult::ACCEPTED);
    ASSERT_TRUE(fixture.fake->wait_for_command_count(1, 1s));
    EXPECT_DOUBLE_EQ(fixture.fake->last_position_command(), 0.0);

    EXPECT_EQ(fixture.worker.submit(EefCommand::CLOSE), SubmitResult::ACCEPTED);
    ASSERT_TRUE(fixture.fake->wait_for_command_count(2, 1s));
    EXPECT_DOUBLE_EQ(fixture.fake->last_position_command(), 1.0);

    EXPECT_EQ(fixture.worker.submit(EefCommand::SET_POSITION, 0.5), SubmitResult::ACCEPTED);
    ASSERT_TRUE(fixture.fake->wait_for_command_count(3, 1s));
    EXPECT_DOUBLE_EQ(fixture.fake->last_position_command(), 0.5);
}

TEST(EefWorkerMailbox, RejectsInvalidAndOutOfRangeCommands) {
    WorkerFixture fixture;
    fixture.activate_ready();

    EXPECT_EQ(fixture.worker.submit(static_cast<EefCommand>(99)), SubmitResult::INVALID_COMMAND);
    EXPECT_EQ(fixture.worker.submit(EefCommand::SET_POSITION, -0.001), SubmitResult::OUT_OF_RANGE);
    EXPECT_EQ(fixture.worker.submit(EefCommand::SET_POSITION, 1.001), SubmitResult::OUT_OF_RANGE);
    EXPECT_EQ(fixture.worker.submit(EefCommand::SET_POSITION, std::numeric_limits<double>::quiet_NaN()),
        SubmitResult::OUT_OF_RANGE);
    EXPECT_EQ(fixture.fake->command_count.load(), 0);
}

TEST(EefWorkerMailbox, StopHasPriorityUntilConsumed) {
    WorkerFixture fixture;
    fixture.fake->block_refresh();
    fixture.activate_ready();
    EXPECT_TRUE(fixture.fake->wait_for_refresh(1s));

    EXPECT_EQ(fixture.worker.submit(EefCommand::STOP), SubmitResult::ACCEPTED);
    EXPECT_EQ(fixture.worker.submit(EefCommand::OPEN), SubmitResult::BUSY);
    fixture.fake->release_refresh();
    EXPECT_TRUE(fixture.fake->wait_for_stop_count(1, 1s));
}

TEST(EefWorkerMailbox, TracksFeedbackAndClearsState) {
    WorkerFixture fixture;
    fixture.activate_ready();
    ASSERT_TRUE(fixture.fake->wait_for_refresh_count(1, 1s));
    ASSERT_TRUE(fixture.worker.last_position().has_value());
    EXPECT_DOUBLE_EQ(*fixture.worker.last_position(), 0.5);

    fixture.worker.shutdown();
    EXPECT_FALSE(fixture.worker.is_active());
    EXPECT_FALSE(fixture.worker.last_position().has_value());
}

TEST(EefWorker, SeparatesInactiveHomingReadyCommandAndDeactivate) {
    WorkerFixture fixture;
    EXPECT_EQ(fixture.worker.state(), EefWorkerState::INACTIVE);
    EXPECT_EQ(fixture.worker.submit(EefCommand::OPEN), SubmitResult::NOT_ACTIVE);

    fixture.worker.request_activate();
    ASSERT_TRUE(fixture.fake->wait_for_activate(1s));
    EXPECT_EQ(fixture.worker.state(), EefWorkerState::HOMING);
    EXPECT_EQ(fixture.worker.submit(EefCommand::CLOSE), SubmitResult::NOT_ACTIVE);

    fixture.fake->release_activate();
    ASSERT_TRUE(fixture.worker.wait_for_state(EefWorkerState::READY, 1s));
    EXPECT_TRUE(fixture.worker.hardware_ready());
    EXPECT_TRUE(fixture.worker.is_active());

    EXPECT_EQ(fixture.worker.submit(EefCommand::SET_POSITION, 0.25), SubmitResult::ACCEPTED);
    ASSERT_TRUE(fixture.fake->wait_for_command_count(1, 1s));
    EXPECT_DOUBLE_EQ(fixture.fake->last_position_command(), 0.25);

    EXPECT_EQ(fixture.worker.submit(EefCommand::STOP), SubmitResult::ACCEPTED);
    ASSERT_TRUE(fixture.fake->wait_for_stop_count(1, 1s));

    fixture.worker.request_deactivate();
    ASSERT_TRUE(fixture.worker.wait_for_state(EefWorkerState::INACTIVE, 1s));
    EXPECT_FALSE(fixture.worker.is_active());
    EXPECT_FALSE(fixture.worker.hardware_ready());
    EXPECT_EQ(fixture.fake->deactivate_count.load(), 1);

    fixture.worker.shutdown();
    EXPECT_EQ(fixture.worker.state(), EefWorkerState::SHUTDOWN);
    EXPECT_EQ(fixture.fake->cleanup_count.load(), 1);
}

TEST(EefWorker, ActivationFailureEntersSafeErrorWithoutActivatingCommands) {
    WorkerFixture fixture;
    fixture.worker.request_activate();
    ASSERT_TRUE(fixture.fake->wait_for_activate(1s));
    fixture.fake->release_activate(true);

    ASSERT_TRUE(fixture.worker.wait_for_state(EefWorkerState::ERROR, 1s));
    EXPECT_FALSE(fixture.worker.is_active());
    EXPECT_FALSE(fixture.worker.hardware_ready());
}

TEST(EefWorker, ThreeConsecutiveFeedbackFailuresSafelyDeactivate) {
    WorkerFixture fixture;
    fixture.fake->fail_refresh(true);
    fixture.activate_ready();

    ASSERT_TRUE(fixture.worker.wait_for_state(EefWorkerState::ERROR, 2s));
    EXPECT_GE(fixture.fake->refresh_count.load(), 3);
    EXPECT_EQ(fixture.fake->deactivate_count.load(), 1);
    EXPECT_FALSE(fixture.worker.is_active());
    EXPECT_FALSE(fixture.worker.hardware_ready());
}

TEST(EefWorker, DeactivateCancelsHomingAndReturnsInactiveWithoutError) {
    WorkerFixture fixture;
    fixture.worker.request_activate();
    ASSERT_TRUE(fixture.fake->wait_for_activate(1s));
    ASSERT_EQ(fixture.worker.state(), EefWorkerState::HOMING);

    fixture.worker.request_deactivate();
    ASSERT_TRUE(fixture.fake->wait_for_cancellation(1s));
    ASSERT_TRUE(fixture.worker.wait_for_state(EefWorkerState::INACTIVE, 1s));

    EXPECT_NE(fixture.worker.state(), EefWorkerState::ERROR);
    EXPECT_FALSE(fixture.worker.is_active());
    EXPECT_FALSE(fixture.worker.hardware_ready());
    EXPECT_EQ(fixture.fake->deactivate_count.load(), 1);
}

TEST(EefWorker, ShutdownCancelsHomingAndJoinsWithoutHomingTimeout) {
    WorkerFixture fixture;
    fixture.worker.request_activate();
    ASSERT_TRUE(fixture.fake->wait_for_activate(1s));
    ASSERT_EQ(fixture.worker.state(), EefWorkerState::HOMING);

    const auto started_at = std::chrono::steady_clock::now();
    fixture.worker.shutdown();
    const auto elapsed = std::chrono::steady_clock::now() - started_at;

    EXPECT_LT(elapsed, 1s);
    EXPECT_EQ(fixture.worker.state(), EefWorkerState::SHUTDOWN);
    EXPECT_TRUE(fixture.fake->wait_for_cancellation(10ms));
    EXPECT_EQ(fixture.fake->cleanup_count.load(), 1);
    EXPECT_FALSE(fixture.worker.is_active());
    EXPECT_FALSE(fixture.worker.hardware_ready());
}

TEST(EefController, RealtimeUpdateIsHardwareIndependentNoOp) {
    EefController controller;
    const rclcpp::Time time(0, 0, RCL_ROS_TIME);
    const auto period = rclcpp::Duration::from_seconds(0.005);

    for(int iteration = 0; iteration < 1000; ++iteration) {
        EXPECT_EQ(controller.update(time, period), controller_interface::return_type::OK);
    }
}

} // namespace
} // namespace tomato_picker_eef
