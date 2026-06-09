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
 * @file drive_controller.hpp
 * @brief CA-1 drive controller – pure C++ FSM logic, no ROS 2 dependencies.
 *
 * DriveController implements the CiA 402 bring-up sequence and fault recovery
 * as a command generator: given the current Statusword, step() returns the
 * ordered list of OD writes the caller must execute to advance the state
 * machine.  The caller is responsible for performing the actual OD access
 * (e.g. via ROS 2 service calls in drive_controller_node.cpp).
 */

#pragma once

#include <cstdint>
#include <vector>

#include "ca1_motor_ctrl/cia402.hpp"

namespace ca1_motor_ctrl
{

/// A single object-dictionary write command returned by DriveController::step().
struct OdCommand
{
  uint16_t index;
  uint8_t subindex;
  int32_t value;
};

/**
 * @brief Pure C++ CiA 402 drive FSM.  No ROS 2 headers included.
 *
 * Typical usage:
 * @code
 * DriveController ctrl;
 * for (;;) {
 *   int32_t sw = od_read(Cia402Index::STATUSWORD, 0);
 *   for (auto & cmd : ctrl.step(sw))
 *     od_write(cmd.index, cmd.subindex, cmd.value);
 * }
 * @endcode
 */
class DriveController
{
public:
  DriveController() = default;

  /**
   * @brief Decode a raw Statusword value into a DriveState.
   *
   * Uses CiA 402 Table 30 masked comparisons.  States are tested from most
   * specific to least specific so that FAULT_REACTION_ACTIVE (0x000F masked
   * by 0x4F) is never misidentified as FAULT (0x0008 masked by 0x4F).
   */
  static DriveState decode_statusword(int32_t sw);

  /**
   * @brief Advance the FSM by one cycle.
   *
   * Decodes @p statusword, updates the internal state, and returns the
   * ordered list of OD writes to execute in order to progress toward
   * Operation Enabled (or recover from a fault).
   *
   * @param statusword  Raw value read from OD index 0x6041.
   * @return            Ordered list of OD write commands to execute.
   */
  std::vector<OdCommand> step(int32_t statusword);

  /// Current drive state (updated on each call to step()).
  DriveState state() const;

  /// True when the last call to step() caused a state transition.
  bool state_changed() const;

  /// Override the target velocity written in OPERATION_ENABLED (default 100 RPM).
  void set_target_velocity(int32_t rpm);

  /// Set profile acceleration written in SWITCHED_ON (default 1000 RPM/s).
  void set_acceleration(int32_t accel_rpm_per_s);

  /// Set profile deceleration written in SWITCHED_ON (default 1000 RPM/s).
  void set_deceleration(int32_t decel_rpm_per_s);

  /**
   * @brief Enable or disable operation.
   *
   * When @p enable is false the FSM holds at Switched On (motor de-energised)
   * instead of advancing to Operation Enabled.  Calling with @p enable true
   * resumes normal bring-up.
   */
  void set_enable(bool enable);

  /// Current enable flag (true → pursue Operation Enabled, false → hold at Switched On).
  bool is_enabled() const;

private:
  DriveState state_{DriveState::UNKNOWN};
  DriveState prev_state_{DriveState::UNKNOWN};
  int32_t target_velocity_{100};
  int32_t accel_{1000};
  int32_t decel_{1000};
  bool enabled_{true};
};

}  // namespace ca1_motor_ctrl
