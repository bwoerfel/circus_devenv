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
 * @file cia402.hpp
 * @brief CiA 402 servo drive protocol – types, constants, and exceptions.
 *
 * Shared by the drive simulator and the drive controller.  Contains only
 * protocol-level definitions; no simulator or ROS 2 dependencies.
 *
 * Object indices (Cia402Index)
 * ----------------------------
 * | Index  | Sub | Name                       | Access |
 * |--------|-----|----------------------------|--------|
 * | 0x6040 |  0  | Controlword                | RW     |
 * | 0x6041 |  0  | Statusword                 | RO     |
 * | 0x6060 |  0  | Modes of operation         | RW     |
 * | 0x6061 |  0  | Modes of operation display | RO     |
 * | 0x60FF |  0  | Target velocity            | RW     |
 * | 0x606C |  0  | Velocity actual value      | RO     |
 * | 0x6064 |  0  | Position actual value      | RO     |
 * | 0x607A |  0  | Target position            | RW     |
 * | 0x603F |  0  | Error code                 | RO     |
 * | 0x6080 |  0  | Max motor speed (sim)      | RW     |
 * | 0x6083 |  0  | Profile acceleration (sim) | RW     |
 * | 0x6084 |  0  | Profile deceleration (sim) | RW     |
 *
 * State transition sequence to reach Operation Enabled:
 *   1. Write Controlword::SHUTDOWN         → Ready to Switch On
 *   2. Write Controlword::SWITCH_ON        → Switched On
 *   3. Write Controlword::ENABLE_OPERATION → Operation Enabled  ← motor runs
 */

#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace ca1_motor_ctrl
{

// ---------------------------------------------------------------------------
// Basic types
// ---------------------------------------------------------------------------

/// OD entry address: 16-bit index + 8-bit subindex packed into a 32-bit key.
using OdAddress = uint32_t;

/// Raw OD value (all objects are represented as 32-bit integers here).
using OdValue = int32_t;

// ---------------------------------------------------------------------------
// CiA 402 object indices (standard)
// ---------------------------------------------------------------------------
namespace Cia402Index
{
static constexpr uint16_t CONTROLWORD = 0x6040;
static constexpr uint16_t STATUSWORD = 0x6041;
static constexpr uint16_t MODES_OF_OPERATION = 0x6060;
static constexpr uint16_t MODES_OF_OPERATION_DISPLAY = 0x6061;
static constexpr uint16_t TARGET_VELOCITY = 0x60FF;
static constexpr uint16_t VELOCITY_ACTUAL = 0x606C;
static constexpr uint16_t POSITION_ACTUAL = 0x6064;
static constexpr uint16_t TARGET_POSITION = 0x607A;
static constexpr uint16_t PROFILE_ACCELERATION = 0x6083;
static constexpr uint16_t PROFILE_DECELERATION = 0x6084;
static constexpr uint16_t MAX_MOTOR_SPEED = 0x6080;  ///< Over-speed protection threshold (RPM)
static constexpr uint16_t ERROR_CODE = 0x603F;
}  // namespace Cia402Index

// ---------------------------------------------------------------------------
// CiA 402 state enumeration
// ---------------------------------------------------------------------------

/**
 * @brief CiA 402 drive states as defined in the standard.
 *
 * The state is encoded in Statusword bits [6:0] – see DriveController::decode_statusword().
 */
enum class DriveState : uint8_t
{
  NOT_READY_TO_SWITCH_ON = 0,
  SWITCH_ON_DISABLED,
  READY_TO_SWITCH_ON,
  SWITCHED_ON,
  OPERATION_ENABLED,
  QUICK_STOP_ACTIVE,
  FAULT_REACTION_ACTIVE,
  FAULT,
  UNKNOWN
};

/// Returns a human-readable string for a DriveState.
const char * to_string(DriveState state);

// ---------------------------------------------------------------------------
// Controlword command constants (bits [3:0])
// ---------------------------------------------------------------------------
namespace Controlword
{
static constexpr uint16_t SHUTDOWN = 0x0006;             ///< → Ready to Switch On
static constexpr uint16_t SWITCH_ON = 0x0007;            ///< → Switched On
static constexpr uint16_t ENABLE_OPERATION = 0x000F;     ///< → Operation Enabled
static constexpr uint16_t DISABLE_VOLTAGE = 0x0000;      ///< → Switch On Disabled (bit 1 = 0)
static constexpr uint16_t QUICK_STOP = 0x0002;           ///< → Quick Stop Active (bit 2 = 0, bit 1 = 1)
static constexpr uint16_t FAULT_RESET_BIT = 0x0080;      ///< Rising edge resets fault
static constexpr uint16_t HALT = (1u << 8);              ///< Halt motion (profile velocity/position)
}  // namespace Controlword

// ---------------------------------------------------------------------------
// Statusword bit masks
// ---------------------------------------------------------------------------
namespace Statusword
{
static constexpr uint16_t READY_TO_SWITCH_ON = (1u << 0);
static constexpr uint16_t SWITCHED_ON = (1u << 1);
static constexpr uint16_t OPERATION_ENABLED = (1u << 2);
static constexpr uint16_t FAULT = (1u << 3);
static constexpr uint16_t VOLTAGE_ENABLED = (1u << 4);
static constexpr uint16_t QUICK_STOP = (1u << 5);             ///< active-low in some impls
static constexpr uint16_t SWITCH_ON_DISABLED = (1u << 6);
static constexpr uint16_t WARNING = (1u << 7);
static constexpr uint16_t REMOTE = (1u << 9);             ///< Drive accepts fieldbus commands
static constexpr uint16_t TARGET_REACHED = (1u << 10);
}  // namespace Statusword

// ---------------------------------------------------------------------------
// Modes of operation
// ---------------------------------------------------------------------------
namespace ModeOfOperation
{
static constexpr int8_t PROFILE_POSITION = 1;
static constexpr int8_t PROFILE_VELOCITY = 3;
static constexpr int8_t HOMING = 6;
static constexpr int8_t CYCLIC_SYNC_POS = 8;
static constexpr int8_t CYCLIC_SYNC_VEL = 9;
static constexpr int8_t CYCLIC_SYNC_TOR = 10;
}  // namespace ModeOfOperation

// ---------------------------------------------------------------------------
// Error codes (stored in ERROR_CODE 0x603F, subset of DS301 Annex C)
// ---------------------------------------------------------------------------
namespace ErrorCode
{
static constexpr uint32_t GENERIC = 0x3210u;       ///< Generic / overcurrent (manufacturer)
static constexpr uint32_t OVERSPEED = 0x8480u;     ///< Speed too high (DS301 §Annex C)
static constexpr uint32_t SENSOR_TIMEOUT = 0xFF01u; ///< Fieldbus watchdog / sensor timeout (mfr)
}  // namespace ErrorCode

// ---------------------------------------------------------------------------
// Exception type
// ---------------------------------------------------------------------------

/**
 * @brief Thrown when an OD access violates the specification.
 *
 * Carries the SDO abort code for compatibility with real drives.
 */
class OdAccessError : public std::runtime_error
{
public:
  explicit OdAccessError(const std::string & msg, uint32_t abort_code = 0)
  : std::runtime_error(msg), abort_code_(abort_code) {}

  uint32_t abort_code() const {return abort_code_;}

private:
  uint32_t abort_code_;
};

}  // namespace ca1_motor_ctrl
