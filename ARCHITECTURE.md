# Architecture

## Overview

`ca1_motor_ctrl` is a ROS 2 Humble package for the CA-1 cooking station motor
controller.  It pairs a software-in-the-loop (SITL) CiA 402 servo drive
simulator with a pure C++ FSM controller, connected by two ROS 2 services that
mimic CANopen / EtherCAT CoE object-dictionary access.  A lightweight Python
web UI provides real-time telemetry and fault injection from any browser.

---

## Directory Layout

```
ca1_motor_ctrl/
├── include/ca1_motor_ctrl/
│   ├── cia402.hpp                 # CiA 402 protocol types, OD indices, Controlword/
│   │                              #   Statusword constants, OdAccessError
│   ├── drive_simulator.hpp        # DriveSimulator class API (no ROS 2 headers)
│   ├── drive_controller.hpp       # DriveController FSM API  (no ROS 2 headers)
│   └── logging.hpp                # Custom timestamped log handler
├── src/
│   ├── drive_simulator.cpp        # OD map, state machine, physics, fault injection
│   ├── drive_simulator_node.cpp   # ROS 2 wrapper: OD services + 100 Hz + 1 Hz timers
│   ├── drive_controller.cpp       # Pure C++ FSM: step() → OD command list
│   ├── drive_controller_node.cpp  # ROS 2 wrapper: FSM thread, pub/sub
│   └── web_ui.py                  # Browser UI on :8080 (Python, SSE at 10 Hz)
├── srv/
│   ├── OdRead.srv                 # Read an OD entry  (index / subindex → value)
│   └── OdWrite.srv                # Write an OD entry (index / subindex / value)
├── test/
│   ├── test_drive_simulator.cpp   # GTest: OD access, state machine, physics, ramps
│   └── test_drive_controller.cpp  # GTest: FSM commands, state decoding, fault recovery
├── docker/
│   └── Dockerfile                 # Dev image (osrf/ros:humble-desktop base)
├── .devcontainer/
│   └── devcontainer.json          # VS Code Dev Container config
└── .github/workflows/
    ├── ci.yml                     # Build + test on every push / PR
    └── docker-publish.yml         # Publish dev image to ghcr.io on Dockerfile changes
```

---

## Component Topology

```
  Browser
  (HTTP/SSE)
      │
      ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  web_ui  (:8080)  [split-screen: controller left, simulator right]  │
 │   SUB  /ca1/ctrl_events              std_msgs/String            │
 │   SUB  /ca1/sim_events               std_msgs/String            │
 │   SUB  /ca1/drive_state              std_msgs/String            │
 │   SUB  /ca1/velocity_actual          std_msgs/Int32             │
 │   SUB  /ca1/position_actual          std_msgs/Int32             │
 │   PUB  /ca1/cmd_velocity             std_msgs/Int32             │
 │   CLI  /ca1/inject_fault             std_srvs/Trigger           │
 │   CLI  /ca1/inject_fault_overspeed   std_srvs/Trigger           │
 │   CLI  /ca1/inject_fault_sensor_timeout std_srvs/Trigger        │
 └──────────────────────────────┬──────────────────────────────────┘
                                │  topics
                                ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  drive_controller_node                                          │
 │   SUB  /ca1/cmd_velocity     std_msgs/Int32                    │
 │   PUB  /ca1/drive_state      std_msgs/String                   │
 │   PUB  /ca1/velocity_actual  std_msgs/Int32                    │
 │   PUB  /ca1/position_actual  std_msgs/Int32                    │
 │   CLI  /ca1/od_read          ca1_motor_ctrl/OdRead             │
 │   CLI  /ca1/od_write         ca1_motor_ctrl/OdWrite            │
 └──────────────────────────────┬──────────────────────────────────┘
                                │  services
                                ▼
 ┌─────────────────────────────────────────────────────────────────┐
 │  drive_simulator_node                                           │
 │   SRV  /ca1/od_read                  ca1_motor_ctrl/OdRead     │
 │   SRV  /ca1/od_write                 ca1_motor_ctrl/OdWrite    │
 │   SRV  /ca1/inject_fault             std_srvs/Trigger          │
 │   SRV  /ca1/inject_fault_overspeed   std_srvs/Trigger          │
 │   SRV  /ca1/inject_fault_sensor_timeout std_srvs/Trigger       │
 └─────────────────────────────────────────────────────────────────┘
```

---

## Node Reference

### drive_simulator_node

| Aspect | Detail |
|---|---|
| Executable | `drive_simulator_node` |
| Node name | `drive_simulator` |
| C++ class | `DriveSimulatorNode` (wraps `DriveSimulator`) |
| Thread model | Single ROS executor thread; all callbacks share `mutex_` |
| **Timers** | |
| Physics | 100 Hz wall timer → `DriveSimulator::update(0.01)` |
| FSM | 1 Hz wall timer → `DriveSimulator::update_fsm()` |
| **Services advertised** | |
| `/ca1/od_read` | `ca1_motor_ctrl/OdRead` — read one OD entry |
| `/ca1/od_write` | `ca1_motor_ctrl/OdWrite` — write one OD entry |
| `/ca1/inject_fault` | `std_srvs/Trigger` — generic fault (error 0x3210) |
| `/ca1/inject_fault_overspeed` | `std_srvs/Trigger` — over-speed fault (error 0x8480); also auto-triggers when \|velocity_actual\| > 0x6080 |
| `/ca1/inject_fault_sensor_timeout` | `std_srvs/Trigger` — sensor/fieldbus watchdog timeout fault (error 0xFF01) |
| **Publishers** | |
| `/ca1/sim_events` | `std_msgs/String` — simulator state/fault events for web UI log |

### drive_controller_node

| Aspect | Detail |
|---|---|
| Executable | `drive_controller_node` |
| Node name | `drive_controller` |
| C++ class | `DriveControllerNode` (wraps `DriveController`) |
| Thread model | FSM loop in `std::thread`; ROS callbacks in `MultiThreadedExecutor`; shared `ctrl_mutex_` |
| **FSM period** | 50 ms |
| **Service clients** | |
| `/ca1/od_read` | `ca1_motor_ctrl/OdRead` — synchronous, 1 s timeout |
| `/ca1/od_write` | `ca1_motor_ctrl/OdWrite` — synchronous, 1 s timeout |
| **Publishers** | |
| `/ca1/drive_state` | `std_msgs/String` — CiA 402 state name on state change |
| `/ca1/velocity_actual` | `std_msgs/Int32` — RPM, every FSM cycle |
| `/ca1/position_actual` | `std_msgs/Int32` — counts, every FSM cycle |
| `/ca1/ctrl_events` | `std_msgs/String` — controller state/fault/error events for web UI log |
| **Subscribers** | |
| `/ca1/cmd_velocity` | `std_msgs/Int32` — target velocity in RPM |

### web_ui

| Aspect | Detail |
|---|---|
| Executable | `web_ui` (Python) |
| Node name | `web_ui` |
| HTTP port | 8080 |
| Layout | Split-screen: left = Drive Controller, right = Drive Simulator |
| **Subscribers** | |
| `/ca1/ctrl_events` | `std_msgs/String` — controller events → left panel log |
| `/ca1/sim_events` | `std_msgs/String` — simulator events → right panel log |
| `/ca1/drive_state` | `std_msgs/String` |
| `/ca1/velocity_actual` | `std_msgs/Int32` |
| `/ca1/position_actual` | `std_msgs/Int32` |
| **Publishers** | |
| `/ca1/cmd_velocity` | `std_msgs/Int32` — target velocity from slider |
| **Service clients** | |
| `/ca1/inject_fault` | `std_srvs/Trigger` — generic fault |
| `/ca1/inject_fault_overspeed` | `std_srvs/Trigger` — over-speed fault |
| `/ca1/inject_fault_sensor_timeout` | `std_srvs/Trigger` — sensor timeout fault |

---

## Build Target Graph

```
cia402.hpp  (header-only)
     │
     ├──► drive_simulator  (static lib, pure C++)
     │         ├──► drive_simulator_node  (exe) ── rclcpp, std_srvs
     │         ├──► drive_controller  (static lib, pure C++)
     │         │         ├──► drive_controller_node  (exe) ── rclcpp, std_msgs
     │         │         └──► test_drive_controller  (GTest)
     │         └──► test_drive_simulator  (GTest)
     │
web_ui.py  (Python install, ros2 run target)
```

The two C++ libraries (`drive_simulator`, `drive_controller`) have no ROS 2
headers, so they can be linked into GTest executables without a running ROS 2
context.

---

## Custom Service Interfaces

### OdRead.srv

```
# Request
uint16 index      # OD index   (e.g. 0x6041 for Statusword)
uint8  subindex   # OD subindex (typically 0x00)
---
# Response
int32  value      # Value read from the OD entry
bool   success    # True on success, false on error
string message    # Human-readable error description (empty on success)
uint32 abort_code # SDO abort code on error (0 on success)
```

### OdWrite.srv

```
# Request
uint16 index      # OD index   (e.g. 0x6040 for Controlword)
uint8  subindex   # OD subindex (typically 0x00)
int32  value      # Value to write
---
# Response
bool   success    # True on success, false on error
string message    # Human-readable error description (empty on success)
uint32 abort_code # SDO abort code on error (0 on success)
```

The `abort_code` field follows the CANopen SDO abort code convention (DS301
§7.2.4.3.17).  Common codes: `0x06090011` (object does not exist),
`0x06010002` (write to read-only object).

---

## Object Dictionary Reference

| Index  | Sub | Name                       | Access | Default  |
|--------|-----|----------------------------|--------|----------|
| 0x6040 |  0  | Controlword                | RW     | 0        |
| 0x6041 |  0  | Statusword                 | RO     | 0        |
| 0x6060 |  0  | Modes of operation         | RW     | 3 (PV)   |
| 0x6061 |  0  | Modes of operation display | RO     | 3 (PV)   |
| 0x6064 |  0  | Position actual value      | RO     | 0        |
| 0x606C |  0  | Velocity actual value      | RO     | 0        |
| 0x607A |  0  | Target position            | RW     | 0        |
| 0x60FF |  0  | Target velocity            | RW     | 0        |
| 0x6080 |  0  | Max motor speed            | RW     | 250      |
| 0x6083 |  0  | Profile acceleration       | RW     | 1000     |
| 0x6084 |  0  | Profile deceleration       | RW     | 1000     |
| 0x603F |  0  | Error code                 | RO     | 0        |

Profile acceleration and deceleration units are RPM/s.  The simulator uses
them to ramp `velocity_actual` toward `target_velocity` in `update()`.

---

## Concurrency & Timing Model

```
drive_simulator_node
  ROS executor thread
    ├── od_read  service  callback  (mutex-locked, ~1 µs)
    ├── od_write service  callback  (mutex-locked, ~1 µs)
    ├── inject_fault service callback (mutex-locked, ~1 µs)
    ├── 100 Hz wall timer → DriveSimulator::update()   (mutex-locked, ~1 µs)
    └──   1 Hz wall timer → DriveSimulator::update_fsm() (mutex-locked)

drive_controller_node
  ROS MultiThreadedExecutor
    └── cmd_velocity subscription callback (ctrl_mutex_-locked)
  FSM background thread (50 ms period)
    ├── sync_read  STATUSWORD   (blocks up to 1 s on od_read service)
    ├── DriveController::step() (ctrl_mutex_-locked)
    ├── sync_write per OD command (blocks up to 1 s per call)
    ├── sync_read  VELOCITY_ACTUAL
    └── sync_read  POSITION_ACTUAL

web_ui
  rclpy.spin() daemon thread
    ├── /ca1/ctrl_events subscription → _ctrl_events deque (left panel)
    ├── /ca1/sim_events  subscription → _sim_events deque  (right panel)
    ├── /ca1/drive_state, /ca1/velocity_actual, /ca1/position_actual subscriptions
    └── fault service async calls (fire-and-forget)
  ThreadingMixIn HTTP server (one thread per SSE client)
    └── SSE stream: status() snapshot at 10 Hz → browser
```

Key invariants:
- The simulator physics callback (~1 µs) holds the mutex for less than 1 tick
  period, so service callbacks never starve.
- FSM state machine transitions in the simulator are applied at 1 Hz
  (`update_fsm()`), decoupling state-machine latency from physics rate — this
  mimics a real drive whose scan cycle is slower than the physics model.
- The controller FSM thread calls synchronous service RPCs; these block the
  thread (not the ROS executor) so topic subscriptions remain responsive.
- `DriveController` (pure C++) is guarded by `ctrl_mutex_` — the FSM thread
  and the `cmd_velocity` callback never race on controller state.
