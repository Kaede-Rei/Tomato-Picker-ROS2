#include <gtest/gtest.h>

#include "tomato_picker_eef/eef_model.hpp"

#include <chrono>
#include <limits>
#include <optional>

namespace tomato_picker_eef {
namespace {

TEST(EefMapping, MatchesPythonGoldenValues) {
    EXPECT_DOUBLE_EQ(command_to_motor_position(0.0), 0.0);
    EXPECT_DOUBLE_EQ(command_to_motor_position(1.0), -5.23);
    EXPECT_DOUBLE_EQ(command_to_motor_position(0.5), -2.615);
    EXPECT_DOUBLE_EQ(command_to_motor_position(kEefCommandMinimum), kEefMotorOpenPosition);
    EXPECT_DOUBLE_EQ(command_to_motor_position(kEefCommandMaximum), kEefMotorClosedPosition);
}

TEST(EefMapping, ClampsAtCapabilityBoundary) {
    EXPECT_DOUBLE_EQ(command_to_motor_position(-0.25), kEefMotorOpenPosition);
    EXPECT_DOUBLE_EQ(command_to_motor_position(1.25), kEefMotorClosedPosition);
}

TEST(EefMapping, PreservesDirectionOffsetAndFeedbackInverse) {
    EXPECT_DOUBLE_EQ(kEefMotorPositionOffset, 0.0);
    EXPECT_LT(kEefMotorPositionScale, 0.0);
    EXPECT_DOUBLE_EQ(motor_position_to_command(0.0), 0.0);
    EXPECT_DOUBLE_EQ(motor_position_to_command(-5.23), 1.0);
    EXPECT_DOUBLE_EQ(motor_position_to_command(-2.615), 0.5);
    EXPECT_DOUBLE_EQ(motor_position_to_command(0.5), 0.0);
    EXPECT_DOUBLE_EQ(motor_position_to_command(-6.0), 1.0);
}

TEST(EefTorquePos, MatchesPythonGoldenRawCalculations) {
    const auto velocity = calculate_velocity_limit();
    const auto current = calculate_current_limit(1.0);
    ASSERT_TRUE(velocity.has_value());
    ASSERT_TRUE(current.has_value());
    EXPECT_EQ(*velocity, 2094U);
    EXPECT_EQ(*current, 1058U);
}

TEST(EefTorquePos, BuildsGoldenPositionCommands) {
    const auto open = make_torque_pos_command(0.0, 1.0);
    const auto middle = make_torque_pos_command(0.5, 1.0);
    const auto closed = make_torque_pos_command(1.0, 1.0);
    ASSERT_TRUE(open.has_value());
    ASSERT_TRUE(middle.has_value());
    ASSERT_TRUE(closed.has_value());
    EXPECT_FLOAT_EQ(open->target_position, 0.0F);
    EXPECT_FLOAT_EQ(middle->target_position, -2.615F);
    EXPECT_FLOAT_EQ(closed->target_position, -5.23F);
    EXPECT_EQ(open->velocity_limit, 2094U);
    EXPECT_EQ(open->current_limit, 1058U);
}

TEST(EefTorquePos, RejectsPositionAndRawRangeViolations) {
    EXPECT_FALSE(make_torque_pos_command(-0.001, 1.0).has_value());
    EXPECT_FALSE(make_torque_pos_command(1.001, 1.0).has_value());
    EXPECT_FALSE(make_torque_pos_command(std::numeric_limits<double>::quiet_NaN(), 1.0).has_value());
    EXPECT_FALSE(calculate_velocity_limit(-1.0).has_value());
    EXPECT_FALSE(calculate_velocity_limit(10000.0).has_value());
    EXPECT_FALSE(calculate_current_limit(-0.1).has_value());
    EXPECT_FALSE(calculate_current_limit(100.0).has_value());
    EXPECT_FALSE(calculate_current_limit(std::numeric_limits<double>::infinity()).has_value());
}

TEST(EefHoming, CompletesOnlyAboveTorqueThreshold) {
    using namespace std::chrono_literals;
    EefHomingStateMachine state_machine(1.2, 5.0s);
    const auto start = EefHomingStateMachine::Clock::time_point{};
    state_machine.start(start);

    EXPECT_EQ(state_machine.observe(1.2, start + 10ms), HomingDecision::CONTINUE);
    EXPECT_EQ(state_machine.observe(1.2001, start + 20ms), HomingDecision::ZERO_AND_FINISH);
}

TEST(EefHoming, TimesOutWithoutEnteringZeroSequence) {
    using namespace std::chrono_literals;
    EefHomingStateMachine state_machine(1.2, 5.0s);
    const auto start = EefHomingStateMachine::Clock::time_point{};
    state_machine.start(start);

    EXPECT_EQ(state_machine.observe(0.5, start + 4999ms), HomingDecision::CONTINUE);
    EXPECT_EQ(state_machine.observe(2.0, start + 5s), HomingDecision::TIMEOUT);
}

TEST(EefHoming, RejectsMissingAndNonFiniteFeedback) {
    using namespace std::chrono_literals;
    EefHomingStateMachine state_machine(1.2, 5.0s);
    const auto start = EefHomingStateMachine::Clock::time_point{};
    state_machine.start(start);

    EXPECT_EQ(state_machine.observe(std::nullopt, start + 10ms), HomingDecision::FEEDBACK_FAILURE);
    EXPECT_EQ(state_machine.observe(std::numeric_limits<double>::quiet_NaN(), start + 10ms),
        HomingDecision::FEEDBACK_FAILURE);
}

} // namespace
} // namespace tomato_picker_eef
