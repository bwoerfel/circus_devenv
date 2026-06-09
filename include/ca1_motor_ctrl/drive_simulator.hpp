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
 * @file drive_simulator.hpp
 * @brief CiA 402 servo drive simulator.
 *
 * Simulates the CiA 402 state machine and a minimal object dictionary so that
 * a controller node can be developed and tested without real hardware.
 *
 * The interface is intentionally fieldbus-agnostic: callers use
 *   read(index, subindex)  /  write(index, subindex, value)
 * matching the semantics of CANopen SDO or EtherCAT CoE mailbox access.
 *
 * @note The simulator is fully implemented as a SITL (Software-In-The-Loop)
 *       test device.  It provides a complete CiA 402 state machine, physics
 *       update, and fault injection so that a controller can be developed and
 *       validated without real hardware.
 *
 * @see cia402.hpp for the protocol types, constants, and exceptions.
 */

#pragma once

#include <unordered_map>

#include "ca1_motor_ctrl/cia402.hpp"

namespace ca1_motor_ctrl
{

/**
 * @brief Minimal CiA 402 drive simulator.
 *
 * Implements the complete object dictionary, CiA 402 state machine, physics
 * update, and fault injection.  Exposes read() / write() for fieldbus-agnostic
 * access identical to real CANopen / EtherCAT CoE drives.
 *
 * Thread safety: NOT thread-safe.  Wrap in a mutex if used from multiple
 * threads (e.g., a ROS 2 service callback and a simulation update loop).
 */
class DriveSimulator
{
public:
  // -------------------------------------------------------------------------
  // Construction
  // -------------------------------------------------------------------------

  DriveSimulator();
  ~DriveSimulator() = default;

  // Non-copyable, movable.
  DriveSimulator(const DriveSimulator &)            = delete;
  DriveSimulator & operator=(const DriveSimulator &) = delete;
  DriveSimulator(DriveSimulator &&)                 = default;
  DriveSimulator & operator=(DriveSimulator &&)     = default;

  // -------------------------------------------------------------------------
  // Object-dictionary interface (plain C++ – used directly or via ROS srv)
  // -------------------------------------------------------------------------

  /**
   * @brief Read an object-dictionary entry.
   *
   * @param index     OD index (e.g. 0x6041 for Statusword).
   * @param subindex  OD subindex (typically 0x00 for single-value objects).
   * @return          Current value as a signed 32-bit integer.
   * @throws OdAccessError if the entry does not exist or is write-only.
   */
  OdValue read(uint16_t index, uint8_t subindex) const;

  /**
   * @brief Write an object-dictionary entry.
   *
   * Writing to Controlword (0x6040) triggers a state-machine transition.
   *
   * @param index     OD index.
   * @param subindex  OD subindex.
   * @param value     Value to write.
   * @throws OdAccessError if the entry does not exist or is read-only.
   */
  void write(uint16_t index, uint8_t subindex, OdValue value);

  // -------------------------------------------------------------------------
  // Convenience accessors
  // -------------------------------------------------------------------------

  /// Current drive state decoded from the internal statusword.
  DriveState state() const;

  /// True when the drive is in Operation Enabled state.
  bool is_operation_enabled() const;

  /// Human-readable name of the current state.
  const char * state_name() const;

  /// Reset the simulator to power-on defaults.
  void reset();

  /**
   * @brief Inject a generic drive fault (error code 0x3210, overcurrent).
   *
   * Transitions through Fault Reaction Active and settles in Fault state.
   * Clear with a Fault Reset Controlword (bit 7 rising edge).
   */
  void inject_fault();

  /**
   * @brief Inject an over-speed fault (error code 0x8480).
   *
   * Also triggered automatically by update() when |velocity_actual| exceeds
   * the MAX_MOTOR_SPEED OD entry (0x6080).
   */
  void inject_overspeed_fault();

  /**
   * @brief Inject a sensor / fieldbus watchdog timeout fault (error code 0xFF01).
   *
   * Simulates the drive losing contact with its position/velocity sensor or a
   * fieldbus watchdog expiry.
   */
  void inject_sensor_timeout_fault();

  // -------------------------------------------------------------------------
  // Simulation update (call periodically to advance internal physics)
  // -------------------------------------------------------------------------

  /**
   * @brief Advance the simulator by @p dt seconds.
   *
   * Updates velocity_actual and position_actual based on target_velocity when
   * the drive is in Operation Enabled state.
   *
   * @param dt  Time step in seconds (e.g. 0.01 for 100 Hz).
   */
  void update(double dt);

  /**
   * @brief Enable or disable deferred state-machine mode.
   *
   * When enabled, Controlword writes are buffered and only applied when
   * update_fsm() is called.  Use this to simulate a drive whose state
   * machine runs at a lower rate than the physics update.
   *
   * @param deferred  true = deferred mode (call update_fsm() periodically),
   *                  false = immediate mode (default, matches original behaviour).
   */
  void set_fsm_deferred(bool deferred);

  /**
   * @brief Apply any pending Controlword write to the state machine.
   *
   * Has no effect when deferred mode is disabled or when no Controlword has
   * been written since the last call.  Call at whatever rate the simulated
   * drive's state machine should run (e.g. 1 Hz).
   */
  void update_fsm();

private:
  // -------------------------------------------------------------------------
  // Internal helpers
  // -------------------------------------------------------------------------

  /// Pack (index, subindex) into a single map key.
  static OdAddress make_key(uint16_t index, uint8_t subindex)
  {
    return (static_cast<uint32_t>(index) << 8) | subindex;
  }

  /**
   * @brief Process a Controlword write and perform state transitions.
   *
   * Implements the full CiA 402 Table 19 transition table including Quick Stop
   * and Fault Reset.
   *
   * @param cw  New controlword value.
   */
  void process_controlword(uint16_t cw);

  /// Set the statusword to reflect @p new_state.
  void set_state(DriveState new_state);

  // -------------------------------------------------------------------------
  // Object dictionary storage
  // -------------------------------------------------------------------------

  /// Access flags for OD entries.
  enum class Access : uint8_t { RO, WO, RW };

  struct OdEntry
  {
    OdValue value;
    Access access;
  };

  std::unordered_map<OdAddress, OdEntry> od_;

  // -------------------------------------------------------------------------
  // Internal state
  // -------------------------------------------------------------------------

  DriveState state_;
  uint16_t prev_controlword_;        ///< Needed to detect rising edges (fault reset)
  double position_actual_;           ///< Internal floating-point accumulator
  bool deferred_fsm_{false};         ///< When true, FSM transitions wait for update_fsm()
  bool cw_pending_{false};           ///< Controlword written but FSM not yet applied
};

}  // namespace ca1_motor_ctrl
