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
 * @file drive_controller.cpp
 * @brief CA-1 drive controller – pure C++ FSM logic implementation.
 *
 * No ROS 2 headers are included here.  The ROS 2 node wrapper lives in
 * drive_controller_node.cpp.
 */

#include "ca1_motor_ctrl/drive_controller.hpp"

namespace ca1_motor_ctrl
{

// ---------------------------------------------------------------------------
// decode_statusword
// ---------------------------------------------------------------------------

DriveState DriveController::decode_statusword(int32_t sw)
{
  // CiA 402 Table 30 – most-specific masks checked first.
  if ((sw & 0x4F) == 0x0F) {return DriveState::FAULT_REACTION_ACTIVE;}
  if ((sw & 0x4F) == 0x08) {return DriveState::FAULT;}
  if ((sw & 0x4F) == 0x40) {return DriveState::SWITCH_ON_DISABLED;}
  if ((sw & 0x4F) == 0x00) {return DriveState::NOT_READY_TO_SWITCH_ON;}
  if ((sw & 0x6F) == 0x07) {return DriveState::QUICK_STOP_ACTIVE;}
  if ((sw & 0x6F) == 0x27) {return DriveState::OPERATION_ENABLED;}
  if ((sw & 0x6F) == 0x23) {return DriveState::SWITCHED_ON;}
  if ((sw & 0x6F) == 0x21) {return DriveState::READY_TO_SWITCH_ON;}
  return DriveState::UNKNOWN;
}

// ---------------------------------------------------------------------------
// step
// ---------------------------------------------------------------------------

std::vector<OdCommand> DriveController::step(int32_t statusword)
{
  prev_state_ = state_;
  state_ = decode_statusword(statusword);

  switch (state_) {
    case DriveState::SWITCH_ON_DISABLED:
      return {{Cia402Index::CONTROLWORD, 0, Controlword::SHUTDOWN}};

    case DriveState::READY_TO_SWITCH_ON:
      return {{Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON}};

    case DriveState::SWITCHED_ON:
      if (!enabled_) {return {};}
      return {
        {Cia402Index::MODES_OF_OPERATION, 0, ModeOfOperation::PROFILE_VELOCITY},
        {Cia402Index::PROFILE_ACCELERATION, 0, accel_},
        {Cia402Index::PROFILE_DECELERATION, 0, decel_},
        {Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION},
      };

    case DriveState::OPERATION_ENABLED:
      if (!enabled_) {
        // Step back to Switched On (motor de-energised) via SWITCH_ON command.
        return {{Cia402Index::CONTROLWORD, 0, Controlword::SWITCH_ON}};
      }
      return {{Cia402Index::TARGET_VELOCITY, 0, target_velocity_}};

    case DriveState::QUICK_STOP_ACTIVE:
      return {{Cia402Index::CONTROLWORD, 0, Controlword::ENABLE_OPERATION}};

    case DriveState::FAULT:
      return {
        {Cia402Index::CONTROLWORD, 0, Controlword::DISABLE_VOLTAGE},
        {Cia402Index::CONTROLWORD, 0, Controlword::FAULT_RESET_BIT}
      };

    case DriveState::FAULT_REACTION_ACTIVE:
      return {};  // wait; drive auto-transitions to FAULT

    case DriveState::NOT_READY_TO_SWITCH_ON:
      return {};  // wait; drive completes self-test

    default:
      return {};
  }
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

DriveState DriveController::state() const
{
  return state_;
}

bool DriveController::state_changed() const
{
  return state_ != prev_state_;
}

void DriveController::set_target_velocity(int32_t rpm)
{
  target_velocity_ = rpm;
}

void DriveController::set_acceleration(int32_t accel_rpm_per_s)
{
  accel_ = accel_rpm_per_s;
}

void DriveController::set_deceleration(int32_t decel_rpm_per_s)
{
  decel_ = decel_rpm_per_s;
}

void DriveController::set_enable(bool enable)
{
  enabled_ = enable;
}

bool DriveController::is_enabled() const
{
  return enabled_;
}

}  // namespace ca1_motor_ctrl
