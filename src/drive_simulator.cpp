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
 * @file drive_simulator.cpp
 * @brief CiA 402 servo drive simulator implementation.
 *
 * Implements the complete object dictionary, CiA 402 state machine, physics
 * update, and fault injection for use as a SITL test device.
 */

#include "ca1_motor_ctrl/drive_simulator.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace ca1_motor_ctrl
{

namespace
{
// Format an OD index/subindex as "0xNNNN" / "0xNN" for error messages.
std::string hex16(uint16_t v)
{
  char buf[7];
  std::snprintf(buf, sizeof(buf), "0x%04X", static_cast<unsigned>(v));
  return buf;
}
std::string hex8(uint8_t v)
{
  char buf[5];
  std::snprintf(buf, sizeof(buf), "0x%02X", static_cast<unsigned>(v));
  return buf;
}
}  // namespace

// ---------------------------------------------------------------------------
// to_string
// ---------------------------------------------------------------------------

const char * to_string(DriveState state)
{
  switch (state) {
    case DriveState::NOT_READY_TO_SWITCH_ON:  return "Not Ready to Switch On";
    case DriveState::SWITCH_ON_DISABLED:      return "Switch On Disabled";
    case DriveState::READY_TO_SWITCH_ON:      return "Ready to Switch On";
    case DriveState::SWITCHED_ON:             return "Switched On";
    case DriveState::OPERATION_ENABLED:       return "Operation Enabled";
    case DriveState::QUICK_STOP_ACTIVE:       return "Quick Stop Active";
    case DriveState::FAULT_REACTION_ACTIVE:   return "Fault Reaction Active";
    case DriveState::FAULT:                   return "Fault";
    default:                                  return "Unknown";
  }
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

DriveSimulator::DriveSimulator()
: state_(DriveState::SWITCH_ON_DISABLED),
  prev_controlword_(0),
  position_actual_(0.0)
{
  reset();
}

// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------

void DriveSimulator::reset()
{
  od_.clear();

  // Populate the object dictionary with power-on defaults.
  // Format: od_[make_key(index, subindex)] = {value, access}
  od_[make_key(Cia402Index::CONTROLWORD, 0)] = {0, Access::RW};
  od_[make_key(Cia402Index::STATUSWORD, 0)] = {0, Access::RO};
  od_[make_key(Cia402Index::MODES_OF_OPERATION, 0)] = {ModeOfOperation::PROFILE_VELOCITY,
    Access::RW};
  od_[make_key(Cia402Index::MODES_OF_OPERATION_DISPLAY, 0)] = {ModeOfOperation::PROFILE_VELOCITY,
    Access::RO};
  od_[make_key(Cia402Index::TARGET_VELOCITY, 0)] = {0, Access::RW};
  od_[make_key(Cia402Index::VELOCITY_ACTUAL, 0)] = {0, Access::RO};
  od_[make_key(Cia402Index::POSITION_ACTUAL, 0)] = {0, Access::RO};
  od_[make_key(Cia402Index::TARGET_POSITION, 0)] = {0, Access::RW};
  od_[make_key(Cia402Index::PROFILE_ACCELERATION, 0)] = {1000, Access::RW};
  od_[make_key(Cia402Index::PROFILE_DECELERATION, 0)] = {1000, Access::RW};
  od_[make_key(Cia402Index::MAX_MOTOR_SPEED, 0)] = {250, Access::RW};
  od_[make_key(Cia402Index::ERROR_CODE, 0)] = {0, Access::RO};

  state_ = DriveState::SWITCH_ON_DISABLED;
  prev_controlword_ = 0;
  position_actual_ = 0.0;

  set_state(DriveState::SWITCH_ON_DISABLED);
}

// ---------------------------------------------------------------------------
// read
// ---------------------------------------------------------------------------

OdValue DriveSimulator::read(uint16_t index, uint8_t subindex) const
{
  const auto key = make_key(index, subindex);
  const auto it = od_.find(key);

  if (it == od_.end()) {
    throw OdAccessError(
            "OD read: object " + hex16(index) +
            " sub " + hex8(subindex) + " does not exist",
            0x06020000u);
  }

  if (it->second.access == Access::WO) {
    throw OdAccessError(
            "OD read: object " + hex16(index) + " is write-only",
            0x06010001u);
  }

  return it->second.value;
}

// ---------------------------------------------------------------------------
// write
// ---------------------------------------------------------------------------

void DriveSimulator::write(uint16_t index, uint8_t subindex, OdValue value)
{
  const auto key = make_key(index, subindex);
  const auto it = od_.find(key);

  if (it == od_.end()) {
    throw OdAccessError(
            "OD write: object " + hex16(index) +
            " sub " + hex8(subindex) + " does not exist",
            0x06020000u);
  }

  if (it->second.access == Access::RO) {
    throw OdAccessError(
            "OD write: object " + hex16(index) + " is read-only",
            0x06010002u);
  }

  it->second.value = value;

  // Special handling: Controlword drives the state machine.
  if (index == Cia402Index::CONTROLWORD && subindex == 0) {
    if (deferred_fsm_) {
      // In deferred mode the FSM is applied on the next update_fsm() call so
      // that the simulated drive reacts at a lower rate than physics updates.
      cw_pending_ = true;
    } else {
      process_controlword(static_cast<uint16_t>(value));
      prev_controlword_ = static_cast<uint16_t>(value);
    }
  }

  // Keep modes_of_operation_display in sync with modes_of_operation.
  if (index == Cia402Index::MODES_OF_OPERATION && subindex == 0) {
    od_[make_key(Cia402Index::MODES_OF_OPERATION_DISPLAY, 0)].value = value;
  }
}

// ---------------------------------------------------------------------------
// Convenience accessors
// ---------------------------------------------------------------------------

DriveState DriveSimulator::state() const
{
  return state_;
}

bool DriveSimulator::is_operation_enabled() const
{
  return state_ == DriveState::OPERATION_ENABLED;
}

const char * DriveSimulator::state_name() const
{
  return to_string(state_);
}

// ---------------------------------------------------------------------------
// update  (simulation physics)
// ---------------------------------------------------------------------------

void DriveSimulator::update(double dt)
{
  if (!is_operation_enabled()) {
    // Drive is not enabled – velocity ramps to zero.
    OdValue vel = od_[make_key(Cia402Index::VELOCITY_ACTUAL, 0)].value;
    if (vel != 0) {
      // Simple first-order decay when disabled.
      const double decay = 0.9;
      vel = static_cast<OdValue>(std::round(vel * decay));
      od_[make_key(Cia402Index::VELOCITY_ACTUAL, 0)].value = vel;
    }
    return;
  }

  // Ramp actual velocity toward target using profile accel/decel (units/s).
  const double target = static_cast<double>(
    od_[make_key(Cia402Index::TARGET_VELOCITY, 0)].value);
  double vel = static_cast<double>(
    od_[make_key(Cia402Index::VELOCITY_ACTUAL, 0)].value);
  const double accel = static_cast<double>(
    od_[make_key(Cia402Index::PROFILE_ACCELERATION, 0)].value);
  const double decel = static_cast<double>(
    od_[make_key(Cia402Index::PROFILE_DECELERATION, 0)].value);

  if (vel < target) {
    vel = std::min(vel + accel * dt, target);
  } else if (vel > target) {
    vel = std::max(vel - decel * dt, target);
  }

  // Over-speed protection: fault if |velocity_actual| exceeds MAX_MOTOR_SPEED.
  const double limit = static_cast<double>(
    od_[make_key(Cia402Index::MAX_MOTOR_SPEED, 0)].value);
  if (std::abs(vel) > limit) {
    inject_overspeed_fault();
    return;
  }

  od_[make_key(Cia402Index::VELOCITY_ACTUAL, 0)].value =
    static_cast<OdValue>(std::round(vel));

  // Integrate position (velocity units × seconds → position counts).
  position_actual_ += vel * dt;
  od_[make_key(Cia402Index::POSITION_ACTUAL, 0)].value =
    static_cast<OdValue>(std::round(position_actual_));
}

// ---------------------------------------------------------------------------
// set_fsm_deferred / update_fsm
// ---------------------------------------------------------------------------

void DriveSimulator::set_fsm_deferred(bool deferred)
{
  deferred_fsm_ = deferred;
  cw_pending_ = false;
}

void DriveSimulator::update_fsm()
{
  if (!cw_pending_) {return;}
  cw_pending_ = false;
  const uint16_t cw =
    static_cast<uint16_t>(od_[make_key(Cia402Index::CONTROLWORD, 0)].value);
  process_controlword(cw);
  prev_controlword_ = cw;
}

// ---------------------------------------------------------------------------
// set_state  (internal)
// ---------------------------------------------------------------------------

void DriveSimulator::set_state(DriveState new_state)
{
  state_ = new_state;

  // Encode state into Statusword according to CiA 402 Table 30.
  uint16_t sw = 0;
  switch (new_state) {
    case DriveState::NOT_READY_TO_SWITCH_ON:
      // xxxx xxxx x0xx 0000
      sw = 0x0000;
      break;

    case DriveState::SWITCH_ON_DISABLED:
      // xxxx xxxx x1xx 0000
      sw = Statusword::SWITCH_ON_DISABLED;
      break;

    case DriveState::READY_TO_SWITCH_ON:
      // xxxx xxxx x01x 0001
      sw = Statusword::READY_TO_SWITCH_ON | Statusword::QUICK_STOP;
      break;

    case DriveState::SWITCHED_ON:
      // xxxx xxxx x01x 0011
      sw = Statusword::READY_TO_SWITCH_ON |
        Statusword::SWITCHED_ON |
        Statusword::QUICK_STOP |
        Statusword::VOLTAGE_ENABLED;
      break;

    case DriveState::OPERATION_ENABLED:
      // xxxx xxxx x01x 0111
      sw = Statusword::READY_TO_SWITCH_ON |
        Statusword::SWITCHED_ON |
        Statusword::OPERATION_ENABLED |
        Statusword::QUICK_STOP |
        Statusword::VOLTAGE_ENABLED;
      break;

    case DriveState::QUICK_STOP_ACTIVE:
      // xxxx xxxx x00x 0111
      sw = Statusword::READY_TO_SWITCH_ON |
        Statusword::SWITCHED_ON |
        Statusword::OPERATION_ENABLED |
        Statusword::VOLTAGE_ENABLED;
      break;

    case DriveState::FAULT_REACTION_ACTIVE:
      sw = Statusword::READY_TO_SWITCH_ON |
        Statusword::SWITCHED_ON |
        Statusword::OPERATION_ENABLED |
        Statusword::FAULT;
      break;

    case DriveState::FAULT:
      sw = Statusword::FAULT;
      break;

    default:
      sw = 0;
      break;
  }

  od_[make_key(Cia402Index::STATUSWORD, 0)].value = static_cast<OdValue>(sw);
}

// ---------------------------------------------------------------------------
// process_controlword
// ---------------------------------------------------------------------------

void DriveSimulator::process_controlword(uint16_t cw)
{
  // Fault reset: rising edge on bit 7 clears FAULT regardless of other bits.
  // prev_controlword_ still holds the previous value here (write() updates it after us).
  const bool fault_reset = (cw & Controlword::FAULT_RESET_BIT) &&
    !(prev_controlword_ & Controlword::FAULT_RESET_BIT);
  if (fault_reset && state_ == DriveState::FAULT) {
    od_[make_key(Cia402Index::ERROR_CODE, 0)].value = 0;
    set_state(DriveState::SWITCH_ON_DISABLED);
    return;
  }

  // Disable Voltage: Enable Voltage bit (bit 1) clear → Switch On Disabled from any state.
  if ((cw & 0x02) == 0) {
    set_state(DriveState::SWITCH_ON_DISABLED);
    return;
  }

  // State-specific transitions (CiA 402 Table 19).
  switch (state_) {
    case DriveState::SWITCH_ON_DISABLED:
      if ((cw & 0x0F) == 0x06) {
        set_state(DriveState::READY_TO_SWITCH_ON);  // Shutdown
      }
      break;

    case DriveState::READY_TO_SWITCH_ON:
      if ((cw & 0x0F) == 0x07) {
        set_state(DriveState::SWITCHED_ON);  // Switch On
      } else if ((cw & 0x0F) == 0x06) {
        set_state(DriveState::SWITCH_ON_DISABLED);  // Shutdown (back)
      }
      break;

    case DriveState::SWITCHED_ON:
      if ((cw & 0x0F) == 0x0F) {
        set_state(DriveState::OPERATION_ENABLED);  // Enable Operation
      } else if ((cw & 0x0F) == 0x06) {
        set_state(DriveState::READY_TO_SWITCH_ON);  // Shutdown
      } else if ((cw & 0x06) == 0x02) {
        set_state(DriveState::QUICK_STOP_ACTIVE);  // Quick Stop (bit 2=0, bit 1=1)
      }
      break;

    case DriveState::OPERATION_ENABLED:
      if ((cw & 0x0F) == 0x07) {
        set_state(DriveState::SWITCHED_ON);  // Disable Operation
      } else if ((cw & 0x0F) == 0x06) {
        set_state(DriveState::READY_TO_SWITCH_ON);  // Shutdown
      } else if ((cw & 0x06) == 0x02) {
        set_state(DriveState::QUICK_STOP_ACTIVE);  // Quick Stop (bit 2=0, bit 1=1)
      }
      break;

    case DriveState::QUICK_STOP_ACTIVE:
      // Enable Operation re-enables from Quick Stop (quick_stop_option_code = 5 behaviour).
      if ((cw & 0x0F) == 0x0F) {
        set_state(DriveState::OPERATION_ENABLED);
      }
      break;

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// Fault injection
// ---------------------------------------------------------------------------

void DriveSimulator::inject_fault()
{
  od_[make_key(Cia402Index::ERROR_CODE, 0)].value =
    static_cast<OdValue>(ErrorCode::GENERIC);
  set_state(DriveState::FAULT_REACTION_ACTIVE);
  set_state(DriveState::FAULT);
}

void DriveSimulator::inject_overspeed_fault()
{
  od_[make_key(Cia402Index::ERROR_CODE, 0)].value =
    static_cast<OdValue>(ErrorCode::OVERSPEED);
  set_state(DriveState::FAULT_REACTION_ACTIVE);
  set_state(DriveState::FAULT);
}

void DriveSimulator::inject_sensor_timeout_fault()
{
  od_[make_key(Cia402Index::ERROR_CODE, 0)].value =
    static_cast<OdValue>(ErrorCode::SENSOR_TIMEOUT);
  set_state(DriveState::FAULT_REACTION_ACTIVE);
  set_state(DriveState::FAULT);
}

}  // namespace ca1_motor_ctrl
