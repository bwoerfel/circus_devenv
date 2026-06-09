# Project context for Claude Code

## What this is
ROS 2 package for a CA-1 cooking station motor controller using a CiA 402
servo drive simulator. 

## ROS 2 target
- Distro: **Humble** 
- Platform: Ubuntu 22.04 Jammy Jellyfish (amd64 / aarch64)
- Language: **C++17**
- Use devcontainer instead of local installation

## Architecture
Two ROS 2 nodes:
- `drive_simulator_node` — wraps `DriveSimulator`, exposes OD access via services
- `drive_controller_node` — the controller node

The simulator and controller communicate via two ROS 2 services:
- `/ca1/od_read`  → `ca1_motor_ctrl/srv/OdRead`
- `/ca1/od_write` → `ca1_motor_ctrl/srv/OdWrite`

## What's done
- `include/ca1_motor_ctrl/cia402.hpp` — CiA 402 protocol types, OD indices, state constants
- `include/ca1_motor_ctrl/drive_simulator.hpp` — `DriveSimulator` class API
- `src/drive_simulator.cpp` — OD map, full state machine, update(), fault injection — all complete
- `src/drive_simulator_node.cpp` — ROS 2 node with services + 100 Hz physics + 1 Hz FSM — complete
- `srv/OdRead.srv` / `srv/OdWrite.srv` — service definitions — complete
- `test/test_drive_simulator.cpp` — GTest unit tests (all passing) — complete
- `CMakeLists.txt` / `package.xml` — build system — complete

## What still needs to be implemented (user's task)

### `src/drive_controller.cpp`
Implement the ROS 2 controller node:
- Wait for simulator services to be available
- Execute the bring-up sequence (Shutdown → Switch On → Enable Operation)
- Verify each transition via Statusword reads
- Command target velocity via 0x60FF writes
- Handle errors gracefully

The simulator is a fully-functional SITL device — state machine, physics, and fault injection
are all implemented.  Use `inject_fault()` (via a future service, or extend the simulator node)
to test fault handling in your controller.  Your task is `drive_controller.cpp` only.

## Key types and constants (all in namespace `ca1_motor_ctrl`)
- `Cia402Index::CONTROLWORD` = 0x6040, `STATUSWORD` = 0x6041, `TARGET_VELOCITY` = 0x60FF
- `Controlword::SHUTDOWN` = 0x0006, `SWITCH_ON` = 0x0007, `ENABLE_OPERATION` = 0x000F
- `DriveState::SWITCH_ON_DISABLED`, `READY_TO_SWITCH_ON`, `SWITCHED_ON`, `OPERATION_ENABLED`
- `OdAccessError` — exception with `.abort_code()`

## Build & test
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ca1_motor_ctrl
colcon test --packages-select ca1_motor_ctrl
colcon test-result --verbose
```

## Test status baseline
All tests should pass on a clean build.

## Dev environment
- Docker-based: `docker/Dockerfile` (osrf/ros:humble-desktop base image)
- VS Code Dev Container: `.devcontainer/devcontainer.json`
- CI: `.github/workflows/ci.yml` (build + test on every push)
- Open the repo in VS Code and click "Reopen in Container" — everything is set up automatically
- Workspace inside container: `/home/newton/ros2_ws/`
- Package source mounted at: `/home/newton/ros2_ws/src/ca1_motor_ctrl/`

## Code Style & Standards
- Style Guide: ROS2 C++ Style Guide (based on Google C++ Style)

## Documentation Standard
- Format: Doxygen (Qt style / JSDoc style preferred)
- Requirements: Every custom ROS2 interface (msg/srv/action) must have inline comments.
- Architecture reference: `ARCHITECTURE.md` at repo root — update after structural changes.

## Building API docs (Sphinx + Breathe + Exhale + Furo)
```bash
# Inside devcontainer – single command builds everything
./build_docs.sh

# Optional: live-reload server on http://localhost:8000
./build_docs.sh --serve

# Manual steps (equivalent)
pip install -r docs/requirements.txt   # sphinx, breathe, exhale, furo
make -C docs html                      # Exhale runs Doxygen, then Sphinx builds HTML

# Output
docs/build/html/index.html
```
The pipeline: Exhale invokes Doxygen on `include/ca1_motor_ctrl/` → XML fed into
Breathe → Sphinx renders Furo-themed HTML with full C++ API tree.