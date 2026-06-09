// Copyright 2026 Benjamin Woerfel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file test_drive_controller.cpp
 * @brief Unit tests for DriveController FSM logic.
 *
 * DriveController::step() is a pure function of the statusword — no ROS 2
 * or simulator required.  Tests verify that the correct OD commands are
 * returned for each drive state.
 *
 * Run with:
 *   colcon test --packages-select ca1_motor_ctrl
 *   colcon test-result --verbose
 */

#include <gtest/gtest.h>
#include "ca1_motor_ctrl/drive_controller.hpp"

using namespace ca1_motor_ctrl;  // NOLINT(build/namespaces)

// ---------------------------------------------------------------------------
// Statusword helpers  (match DriveSimulator::set_state encoding)
// ---------------------------------------------------------------------------

// Not Ready to Switch On: all zero
static constexpr int32_t SW_NOT_READY_TO_SWITCH_ON = 0x0000;
// Switch On Disabled:  bit 6
static constexpr int32_t SW_SWITCH_ON_DISABLED = 0x0040;
// Ready to Switch On:  bit 5 (QUICK_STOP) | bit 0
static constexpr int32_t SW_READY_TO_SWITCH_ON = 0x0021;
// Switched On:         bits 5,4,1,0
static constexpr int32_t SW_SWITCHED_ON = 0x0033;
// Operation Enabled:   bits 5,4,2,1,0
static constexpr int32_t SW_OPERATION_ENABLED = 0x0037;
// Quick Stop Active:   bits 4,2,1,0  (bit 5 QUICK_STOP cleared)
static constexpr int32_t SW_QUICK_STOP_ACTIVE = 0x0017;
// Fault:               bit 3
static constexpr int32_t SW_FAULT = 0x0008;
// Fault Reaction Active: bits 3,2,1,0  (FAULT bit set while still executing deceleration)
static constexpr int32_t SW_FAULT_REACTION_ACTIVE = 0x000F;

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class DriveControllerTest : public ::testing::Test
{
protected:
  DriveController ctrl;
};

// ---------------------------------------------------------------------------
// State decoding
// ---------------------------------------------------------------------------

TEST_F(DriveControllerTest, DecodesSwitchOnDisabled)
{
  EXPECT_EQ(
    DriveController::decode_statusword(SW_SWITCH_ON_DISABLED),
    DriveState::SWITCH_ON_DISABLED);
}

TEST_F(DriveControllerTest, DecodesReadyToSwitchOn)
{
  EXPECT_EQ(
    DriveController::decode_statusword(SW_READY_TO_SWITCH_ON),
    DriveState::READY_TO_SWITCH_ON);
}

TEST_F(DriveControllerTest, DecodesSwitchedOn)
{
  EXPECT_EQ(
    DriveController::decode_statusword(SW_SWITCHED_ON),
    DriveState::SWITCHED_ON);
}

TEST_F(DriveControllerTest, DecodesOperationEnabled)
{
  EXPECT_EQ(
    DriveController::decode_statusword(SW_OPERATION_ENABLED),
    DriveState::OPERATION_ENABLED);
}

TEST_F(DriveControllerTest, DecodesQuickStopActive)
{
  EXPECT_EQ(
    DriveController::decode_statusword(SW_QUICK_STOP_ACTIVE),
    DriveState::QUICK_STOP_ACTIVE);
}

TEST_F(DriveControllerTest, DecodesFault)
{
  EXPECT_EQ(
    DriveController::decode_statusword(SW_FAULT),
    DriveState::FAULT);
}

TEST_F(DriveControllerTest, DecodesFaultReactionActive)
{
  EXPECT_EQ(
    DriveController::decode_statusword(SW_FAULT_REACTION_ACTIVE),
    DriveState::FAULT_REACTION_ACTIVE);
}

TEST_F(DriveControllerTest, DecodesNotReadyToSwitchOn)
{
  EXPECT_EQ(
    DriveController::decode_statusword(SW_NOT_READY_TO_SWITCH_ON),
    DriveState::NOT_READY_TO_SWITCH_ON);
}

// ---------------------------------------------------------------------------
// Bring-up sequence commands
// ---------------------------------------------------------------------------

TEST_F(DriveControllerTest, SwitchOnDisabledIssuesShutdown)
{
  const auto cmds = ctrl.step(SW_SWITCH_ON_DISABLED);
  ASSERT_EQ(cmds.size(), 1u);
  EXPECT_EQ(cmds[0].index, Cia402Index::CONTROLWORD);
  EXPECT_EQ(cmds[0].value, static_cast<int32_t>(Controlword::SHUTDOWN));
}

TEST_F(DriveControllerTest, ReadyToSwitchOnIssuesSwitchOn)
{
  const auto cmds = ctrl.step(SW_READY_TO_SWITCH_ON);
  ASSERT_EQ(cmds.size(), 1u);
  EXPECT_EQ(cmds[0].index, Cia402Index::CONTROLWORD);
  EXPECT_EQ(cmds[0].value, static_cast<int32_t>(Controlword::SWITCH_ON));
}

TEST_F(DriveControllerTest, SwitchedOnSetsModeAccelDecelThenEnableOperation)
{
  const auto cmds = ctrl.step(SW_SWITCHED_ON);
  ASSERT_EQ(cmds.size(), 4u);
  // [0] Profile Velocity mode
  EXPECT_EQ(cmds[0].index, Cia402Index::MODES_OF_OPERATION);
  EXPECT_EQ(cmds[0].value, static_cast<int32_t>(ModeOfOperation::PROFILE_VELOCITY));
  // [1] Profile acceleration
  EXPECT_EQ(cmds[1].index, Cia402Index::PROFILE_ACCELERATION);
  EXPECT_GT(cmds[1].value, 0);
  // [2] Profile deceleration
  EXPECT_EQ(cmds[2].index, Cia402Index::PROFILE_DECELERATION);
  EXPECT_GT(cmds[2].value, 0);
  // [3] Enable operation
  EXPECT_EQ(cmds[3].index, Cia402Index::CONTROLWORD);
  EXPECT_EQ(cmds[3].value, static_cast<int32_t>(Controlword::ENABLE_OPERATION));
}

TEST_F(DriveControllerTest, OperationEnabledWritesTargetVelocity)
{
  ctrl.set_target_velocity(150);
  const auto cmds = ctrl.step(SW_OPERATION_ENABLED);
  ASSERT_EQ(cmds.size(), 1u);
  EXPECT_EQ(cmds[0].index, Cia402Index::TARGET_VELOCITY);
  EXPECT_EQ(cmds[0].value, 150);
}

TEST_F(DriveControllerTest, QuickStopActiveReenables)
{
  const auto cmds = ctrl.step(SW_QUICK_STOP_ACTIVE);
  ASSERT_EQ(cmds.size(), 1u);
  EXPECT_EQ(cmds[0].index, Cia402Index::CONTROLWORD);
  EXPECT_EQ(cmds[0].value, static_cast<int32_t>(Controlword::ENABLE_OPERATION));
}

// ---------------------------------------------------------------------------
// Fault recovery
// ---------------------------------------------------------------------------

TEST_F(DriveControllerTest, FaultIssuesDisableVoltageAndFaultReset)
{
  const auto cmds = ctrl.step(SW_FAULT);
  ASSERT_EQ(cmds.size(), 2u);
  EXPECT_EQ(cmds[0].index, Cia402Index::CONTROLWORD);
  EXPECT_EQ(cmds[0].value, static_cast<int32_t>(Controlword::DISABLE_VOLTAGE));
  EXPECT_EQ(cmds[1].index, Cia402Index::CONTROLWORD);
  EXPECT_EQ(cmds[1].value, static_cast<int32_t>(Controlword::FAULT_RESET_BIT));
}

// ---------------------------------------------------------------------------
// State tracking
// ---------------------------------------------------------------------------

TEST_F(DriveControllerTest, StateChangedOnFirstStep)
{
  ctrl.step(SW_SWITCH_ON_DISABLED);
  EXPECT_TRUE(ctrl.state_changed());
  EXPECT_EQ(ctrl.state(), DriveState::SWITCH_ON_DISABLED);
}

TEST_F(DriveControllerTest, FaultReactionActiveReturnsNoCommands)
{
  const auto cmds = ctrl.step(SW_FAULT_REACTION_ACTIVE);
  EXPECT_EQ(cmds.size(), 0u);
}

TEST_F(DriveControllerTest, NotReadyToSwitchOnReturnsNoCommands)
{
  const auto cmds = ctrl.step(SW_NOT_READY_TO_SWITCH_ON);
  EXPECT_EQ(cmds.size(), 0u);
}

TEST_F(DriveControllerTest, StateChangedFalseWhenSameState)
{
  ctrl.step(SW_SWITCH_ON_DISABLED);
  ctrl.step(SW_SWITCH_ON_DISABLED);
  EXPECT_FALSE(ctrl.state_changed());
}
