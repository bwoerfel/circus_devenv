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
 * @file test_drive_simulator.cpp
 * @brief Unit tests for DriveSimulator.
 *
 * Tests cover:
 *   - Object dictionary read/write
 *   - Error handling (non-existent object, RO write, WO read)
 *   - Initial state
 *   - State machine transitions
 *   - Physics update
 *   - Reset
 *
 * Run with:
 *   colcon test --packages-select ca1_motor_ctrl
 *   colcon test-result --verbose
 */

#include <gtest/gtest.h>
#include "ca1_motor_ctrl/drive_simulator.hpp"

using namespace ca1_motor_ctrl;  // NOLINT(build/namespaces)

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class DriveSimulatorTest : public ::testing::Test
{
protected:
  DriveSimulator sim;
};

// ---------------------------------------------------------------------------
// Object dictionary – basic access
// ---------------------------------------------------------------------------

TEST_F(DriveSimulatorTest, ReadStatusword)
{
  // Statusword must be readable at power-on.
  EXPECT_NO_THROW(sim.read(Cia402Index::STATUSWORD, 0));
}

TEST_F(DriveSimulatorTest, WriteControlword)
{
  // Writing Controlword must not throw.
  EXPECT_NO_THROW(sim.write(Cia402Index::CONTROLWORD, 0, 0x0000));
}

TEST_F(DriveSimulatorTest, ReadWriteTargetVelocity)
{
  sim.write(Cia402Index::TARGET_VELOCITY, 0, 1500);
  EXPECT_EQ(sim.read(Cia402Index::TARGET_VELOCITY, 0), 1500);
}

TEST_F(DriveSimulatorTest, ReadWriteModesOfOperation)
{
  sim.write(Cia402Index::MODES_OF_OPERATION, 0, ModeOfOperation::PROFILE_VELOCITY);
  EXPECT_EQ(sim.read(Cia402Index::MODES_OF_OPERATION, 0), ModeOfOperation::PROFILE_VELOCITY);
  // Display object must mirror the write.
  EXPECT_EQ(
    sim.read(Cia402Index::MODES_OF_OPERATION_DISPLAY, 0),
    ModeOfOperation::PROFILE_VELOCITY);
}

// ---------------------------------------------------------------------------
// Object dictionary – error handling
// ---------------------------------------------------------------------------

TEST_F(DriveSimulatorTest, ReadNonExistentObjectThrows)
{
  EXPECT_THROW(sim.read(0x1234, 0), OdAccessError);
}

TEST_F(DriveSimulatorTest, WriteNonExistentObjectThrows)
{
  EXPECT_THROW(sim.write(0x1234, 0, 0), OdAccessError);
}

TEST_F(DriveSimulatorTest, WriteReadOnlyStatuswordThrows)
{
  EXPECT_THROW(sim.write(Cia402Index::STATUSWORD, 0, 0), OdAccessError);
}

TEST_F(DriveSimulatorTest, WriteReadOnlyVelocityActualThrows)
{
  EXPECT_THROW(sim.write(Cia402Index::VELOCITY_ACTUAL, 0, 100), OdAccessError);
}

TEST_F(DriveSimulatorTest, OdAccessErrorCarriesAbortCode)
{
  try {
    sim.read(0xDEAD, 0);
    FAIL() << "Expected OdAccessError";
  } catch (const OdAccessError & e) {
    EXPECT_NE(e.abort_code(), 0u);
  }
}

// ---------------------------------------------------------------------------
// Initial state
// ---------------------------------------------------------------------------

TEST_F(DriveSimulatorTest, InitialStateIsSwitchOnDisabled)
{
  EXPECT_EQ(sim.state(), DriveState::SWITCH_ON_DISABLED);
}

TEST_F(DriveSimulatorTest, InitiallyNotOperationEnabled)
{
  EXPECT_FALSE(sim.is_operation_enabled());
}

TEST_F(DriveSimulatorTest, InitialVelocityActualIsZero)
{
  EXPECT_EQ(sim.read(Cia402Index::VELOCITY_ACTUAL, 0), 0);
}

TEST_F(DriveSimulatorTest, InitialPositionActualIsZero)
{
  EXPECT_EQ(sim.read(Cia402Index::POSITION_ACTUAL, 0), 0);
}

// ---------------------------------------------------------------------------
// State machine – transitions
// ---------------------------------------------------------------------------

TEST_F(DriveSimulatorTest, ShutdownCommandTransitionsToReadyToSwitchOn)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  EXPECT_EQ(sim.state(), DriveState::READY_TO_SWITCH_ON)
    << "After Shutdown command the drive must enter Ready to Switch On.";
}

TEST_F(DriveSimulatorTest, SwitchOnCommandTransitionsToSwitchedOn)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  EXPECT_EQ(sim.state(), DriveState::SWITCHED_ON)
    << "After Switch On command the drive must enter Switched On.";
}

TEST_F(DriveSimulatorTest, EnableOperationTransitionsToOperationEnabled)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  EXPECT_EQ(sim.state(), DriveState::OPERATION_ENABLED)
    << "After Enable Operation the drive must enter Operation Enabled.";
  EXPECT_TRUE(sim.is_operation_enabled());
}

TEST_F(DriveSimulatorTest, DisableVoltageFromAnyStateGoesToSwitchOnDisabled)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::DISABLE_VOLTAGE);
  EXPECT_EQ(sim.state(), DriveState::SWITCH_ON_DISABLED);
}

TEST_F(DriveSimulatorTest, FullBringUpSequenceReflectedInStatusword)
{
  // Bring up
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);

  const auto sw = static_cast<uint16_t>(sim.read(Cia402Index::STATUSWORD, 0));

  EXPECT_TRUE(sw & Statusword::READY_TO_SWITCH_ON);
  EXPECT_TRUE(sw & Statusword::SWITCHED_ON);
  EXPECT_TRUE(sw & Statusword::OPERATION_ENABLED);
  EXPECT_FALSE(sw & Statusword::FAULT);
}

TEST_F(DriveSimulatorTest, QuickStopFromOperationEnabled)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  ASSERT_EQ(sim.state(), DriveState::OPERATION_ENABLED);

  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::QUICK_STOP);
  EXPECT_EQ(sim.state(), DriveState::QUICK_STOP_ACTIVE);
}

TEST_F(DriveSimulatorTest, QuickStopActiveReenableViaEnableOperation)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::QUICK_STOP);
  ASSERT_EQ(sim.state(), DriveState::QUICK_STOP_ACTIVE);

  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  EXPECT_EQ(sim.state(), DriveState::OPERATION_ENABLED);
}

TEST_F(DriveSimulatorTest, InjectFaultTransitionsToDriveFault)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  ASSERT_TRUE(sim.is_operation_enabled());

  sim.inject_fault();
  EXPECT_EQ(sim.state(), DriveState::FAULT);

  const auto sw = static_cast<uint16_t>(sim.read(Cia402Index::STATUSWORD, 0));
  EXPECT_TRUE(sw & Statusword::FAULT);
}

TEST_F(DriveSimulatorTest, FaultResetClearsFaultState)
{
  sim.inject_fault();
  ASSERT_EQ(sim.state(), DriveState::FAULT);

  // Rising edge on bit 7 clears the fault.
  sim.write(Cia402Index::CONTROLWORD, 0, 0x0000);  // ensure bit 7 is low first
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::FAULT_RESET_BIT);
  EXPECT_EQ(sim.state(), DriveState::SWITCH_ON_DISABLED);
}

// ---------------------------------------------------------------------------
// Physics update
// ---------------------------------------------------------------------------

TEST_F(DriveSimulatorTest, UpdateDoesNotMovePositionWhenDisabled)
{
  sim.write(Cia402Index::TARGET_VELOCITY, 0, 1000);
  sim.update(1.0);  // 1 second, but not enabled
  EXPECT_EQ(sim.read(Cia402Index::POSITION_ACTUAL, 0), 0);
}

TEST_F(DriveSimulatorTest, UpdateMovesPositionWhenEnabled)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  ASSERT_TRUE(sim.is_operation_enabled());

  // Raise max speed well above the test velocity so overspeed protection
  // does not interfere with what this test is actually checking (position
  // integration).
  sim.write(Cia402Index::MAX_MOTOR_SPEED, 0, 5000);
  // Large accel so velocity reaches target in one step (instantaneous for test).
  sim.write(Cia402Index::PROFILE_ACCELERATION, 0, 1000000);
  sim.write(Cia402Index::PROFILE_DECELERATION, 0, 1000000);
  sim.write(Cia402Index::TARGET_VELOCITY, 0, 1000);
  sim.update(1.0);
  EXPECT_EQ(sim.read(Cia402Index::POSITION_ACTUAL, 0), 1000);
}

TEST_F(DriveSimulatorTest, VelocityRampsWithProfile)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  ASSERT_TRUE(sim.is_operation_enabled());

  sim.write(Cia402Index::PROFILE_ACCELERATION, 0, 100);  // 100 units/s
  sim.write(Cia402Index::TARGET_VELOCITY, 0, 100);

  sim.update(0.5);  // half-way up the ramp: vel = 50
  EXPECT_EQ(sim.read(Cia402Index::VELOCITY_ACTUAL, 0), 50);

  sim.update(0.5);  // reaches target: vel = 100
  EXPECT_EQ(sim.read(Cia402Index::VELOCITY_ACTUAL, 0), 100);
}

TEST_F(DriveSimulatorTest, VelocityDecelerates)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  ASSERT_TRUE(sim.is_operation_enabled());

  // Snap to 100 then decelerate.
  sim.write(Cia402Index::PROFILE_ACCELERATION, 0, 1000000);
  sim.write(Cia402Index::PROFILE_DECELERATION, 0, 100);
  sim.write(Cia402Index::TARGET_VELOCITY, 0, 100);
  sim.update(0.01);
  ASSERT_EQ(sim.read(Cia402Index::VELOCITY_ACTUAL, 0), 100);

  sim.write(Cia402Index::TARGET_VELOCITY, 0, 0);
  sim.update(0.5);  // half-way down: vel = 50
  EXPECT_EQ(sim.read(Cia402Index::VELOCITY_ACTUAL, 0), 50);
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

TEST_F(DriveSimulatorTest, ResetRestoresInitialState)
{
  sim.write(Cia402Index::TARGET_VELOCITY, 0, 9999);
  sim.reset();
  EXPECT_EQ(sim.state(), DriveState::SWITCH_ON_DISABLED);
  EXPECT_EQ(sim.read(Cia402Index::TARGET_VELOCITY, 0), 0);
  EXPECT_EQ(sim.read(Cia402Index::POSITION_ACTUAL, 0), 0);
}

// ---------------------------------------------------------------------------
// Fault types
// ---------------------------------------------------------------------------

TEST_F(DriveSimulatorTest, InjectOverspeedFaultSetsErrorCodeAndFaultState)
{
  sim.inject_overspeed_fault();
  EXPECT_EQ(sim.state(), DriveState::FAULT);
  EXPECT_EQ(
    static_cast<uint32_t>(sim.read(Cia402Index::ERROR_CODE, 0)),
    ErrorCode::OVERSPEED);
  const auto sw = static_cast<uint16_t>(sim.read(Cia402Index::STATUSWORD, 0));
  EXPECT_TRUE(sw & Statusword::FAULT);
}

TEST_F(DriveSimulatorTest, InjectSensorTimeoutFaultSetsErrorCodeAndFaultState)
{
  sim.inject_sensor_timeout_fault();
  EXPECT_EQ(sim.state(), DriveState::FAULT);
  EXPECT_EQ(
    static_cast<uint32_t>(sim.read(Cia402Index::ERROR_CODE, 0)),
    ErrorCode::SENSOR_TIMEOUT);
}

TEST_F(DriveSimulatorTest, GenericFaultErrorCodeUnchanged)
{
  sim.inject_fault();
  EXPECT_EQ(
    static_cast<uint32_t>(sim.read(Cia402Index::ERROR_CODE, 0)),
    ErrorCode::GENERIC);
}

TEST_F(DriveSimulatorTest, AutoOverspeedFaultWhenVelocityExceedsLimit)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  ASSERT_TRUE(sim.is_operation_enabled());

  // Instantaneous accel, target above default limit (250 RPM).
  sim.write(Cia402Index::PROFILE_ACCELERATION, 0, 1000000);
  sim.write(Cia402Index::TARGET_VELOCITY, 0, 300);
  sim.update(0.01);

  EXPECT_EQ(sim.state(), DriveState::FAULT);
  EXPECT_EQ(
    static_cast<uint32_t>(sim.read(Cia402Index::ERROR_CODE, 0)),
    ErrorCode::OVERSPEED);
}

TEST_F(DriveSimulatorTest, NoOverspeedFaultWhenVelocityBelowLimit)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  ASSERT_TRUE(sim.is_operation_enabled());

  sim.write(Cia402Index::PROFILE_ACCELERATION, 0, 1000000);
  sim.write(Cia402Index::TARGET_VELOCITY, 0, 200);  // within ±250 default limit
  sim.update(0.01);

  EXPECT_EQ(sim.state(), DriveState::OPERATION_ENABLED);
}

TEST_F(DriveSimulatorTest, MaxMotorSpeedIsConfigurableViaOD)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  ASSERT_TRUE(sim.is_operation_enabled());

  // Lower the limit to 100 RPM.
  sim.write(Cia402Index::MAX_MOTOR_SPEED, 0, 100);
  sim.write(Cia402Index::PROFILE_ACCELERATION, 0, 1000000);
  sim.write(Cia402Index::TARGET_VELOCITY, 0, 150);
  sim.update(0.01);

  EXPECT_EQ(sim.state(), DriveState::FAULT);
  EXPECT_EQ(
    static_cast<uint32_t>(sim.read(Cia402Index::ERROR_CODE, 0)),
    ErrorCode::OVERSPEED);
}

// ---------------------------------------------------------------------------
// Bug regression: Disable Voltage and Quick Stop condition checks
// ---------------------------------------------------------------------------

TEST_F(DriveSimulatorTest, DisableVoltageFromQuickStopGoesToSwitchOnDisabled)
{
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::QUICK_STOP);
  ASSERT_EQ(sim.state(), DriveState::QUICK_STOP_ACTIVE);

  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::DISABLE_VOLTAGE);
  EXPECT_EQ(sim.state(), DriveState::SWITCH_ON_DISABLED);
}

TEST_F(DriveSimulatorTest, QuickStopCommandIdentifiedByBits1And2Only)
{
  // 0x0012 = bit 4 | bit 1; Quick Stop fires on (cw & 0x06) == 0x02 (bit 2=0, bit 1=1).
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON);
  sim.write(Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION);
  ASSERT_EQ(sim.state(), DriveState::OPERATION_ENABLED);

  sim.write(Cia402Index::CONTROLWORD, 0, 0x0012);
  EXPECT_EQ(sim.state(), DriveState::QUICK_STOP_ACTIVE);
}

// ---------------------------------------------------------------------------
// to_string
// ---------------------------------------------------------------------------

TEST(DriveStateTest, ToStringReturnsNonEmptyForAllStates)
{
  const DriveState all_states[] = {
    DriveState::NOT_READY_TO_SWITCH_ON,
    DriveState::SWITCH_ON_DISABLED,
    DriveState::READY_TO_SWITCH_ON,
    DriveState::SWITCHED_ON,
    DriveState::OPERATION_ENABLED,
    DriveState::QUICK_STOP_ACTIVE,
    DriveState::FAULT_REACTION_ACTIVE,
    DriveState::FAULT,
    DriveState::UNKNOWN,
  };
  for (const auto s : all_states) {
    EXPECT_STRNE(to_string(s), "") << "to_string returned empty for state " << static_cast<int>(s);
  }
}
