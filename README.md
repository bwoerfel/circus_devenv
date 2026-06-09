# ca1_motor_ctrl

ROS 2 package for the CA-1 cooking station motor controller.

Implements a CiA 402 servo drive simulator with a minimal object-dictionary
interface, mimicking CANopen / EtherCAT CoE fieldbus access over ROS 2 services.

**Target distro:** ROS 2 Humble Hawksbill (LTS, EOL May 2027)
**Platform:** Ubuntu 22.04 Jammy Jellyfish

---

## Dev environment (Docker + VS Code Dev Containers)

### Prerequisites
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (or Docker Engine on Linux)
- [VS Code](https://code.visualstudio.com/) with the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- [GitHub CLI](https://cli.github.com/) (`gh`) — used to authenticate Docker with the private container registry

### One-time authentication (host machine)

The dev image is hosted on the GitHub Container Registry (GHCR).  Log in once so
Docker can pull it:

```bash
# 1. Authenticate gh with your GitHub account (follow the interactive prompts)
gh auth login --scopes read:packages

# 2. Verify it worked
gh auth status
```

The devcontainer uses this session automatically on every subsequent open — no further manual steps needed.

### Setup (first time ~3-5 min)
```bash
git clone <your-repo-url>
cd ca1_motor_ctrl
code .
# VS Code prompts: "Reopen in Container" -> click it
# Image is pulled from ghcr.io, deps install, package builds automatically
```

No ROS 2 installation needed on the host. Everything runs inside the container.

### Manual Docker usage (without VS Code)
```bash
# Authenticate first (requires gh auth login --scopes read:packages, see above)
gh auth token | docker login ghcr.io -u $(gh api user --jq .login) --password-stdin

# Pull the pre-built image from GitHub Container Registry
docker pull ghcr.io/bwoerfel/ca1_motor_ctrl:latest

# Run an interactive shell
docker run -it --rm \
  -v $(pwd):/home/newton/ros2_ws/src/ca1_motor_ctrl \
  ghcr.io/bwoerfel/ca1_motor_ctrl:latest

# Build the image locally (if you need to modify the Dockerfile)
docker build -f docker/Dockerfile -t ca1_motor_ctrl .
```

---

## Package layout

```
ca1_motor_ctrl/
├── .devcontainer/
│   └── devcontainer.json         # VS Code Dev Container config
├── docker/
│   └── Dockerfile                # osrf/ros:humble-desktop base image
├── .github/workflows/
│   ├── ci.yml                    # GitHub Actions: build + test on every push
│   └── docker-publish.yml        # GitHub Actions: build + push dev image to ghcr.io
├── include/ca1_motor_ctrl/
│   ├── cia402.hpp                # CiA 402 protocol types, OD indices, state machine constants
│   ├── drive_simulator.hpp       # Drive simulator class (no ROS 2 dependencies)
│   ├── drive_controller.hpp      # Pure C++ FSM logic (no ROS 2 dependencies)
│   └── logging.hpp               # Custom timestamped log handler
├── src/
│   ├── drive_simulator.cpp       # Simulator: OD map, state machine, physics, fault injection
│   ├── drive_simulator_node.cpp  # ROS 2 node: OD services + 100 Hz tick + 1 Hz FSM
│   ├── drive_controller.cpp      # Pure C++ FSM implementation (library, no ROS 2)
│   ├── drive_controller_node.cpp # ROS 2 node: FSM loop, OD service clients, state/velocity topics
│   └── web_ui.py                 # Minimal web UI (Python HTTP server, port 8080)
├── srv/
│   ├── OdRead.srv                # Service: read an OD entry
│   └── OdWrite.srv               # Service: write an OD entry
├── test/
│   └── test_drive_simulator.cpp  # Unit tests (GTest)
└── .vscode/
    ├── c_cpp_properties.json
    ├── tasks.json
    ├── launch.json
    └── settings.json
```

---

## Architecture

> For the full architecture document — directory layout, component topology,
> node reference, build graph, and timing model — see [ARCHITECTURE.md](ARCHITECTURE.md).

Two ROS 2 nodes communicate via the OD service pair:

```
drive_simulator_node  ←──/ca1/od_read──→  drive_controller_node
                      ←──/ca1/od_write──→
```

The drive controller is split into a **pure C++ library** (no ROS 2 headers) and a
**ROS 2 node wrapper** — the same pattern as the simulator:

| Target | Sources | Role |
|---|---|---|
| `drive_simulator` (lib) | `drive_simulator.cpp` | OD map, state machine, physics |
| `drive_simulator_node` (exe) | `drive_simulator_node.cpp` | ROS 2 services + 100 Hz physics + 1 Hz FSM |
| `drive_controller` (lib) | `drive_controller.cpp` | Pure C++ FSM, `step()` → OD commands |
| `drive_controller_node` (exe) | `drive_controller_node.cpp` | ROS 2 FSM loop + pub/sub |

The simulator runs physics (velocity/position integration) at 100 Hz and applies
state-machine transitions at 1 Hz — mimicking a real drive whose scan cycle is
slower than the physics model.  Fault injection is triggered on demand via the
"Inject Fault" button in the web UI; the controller resets the drive automatically.

---

## Build & run

### Option A — VS Code Dev Container (recommended)

1. Open the repo in VS Code
2. **F1** → `Dev Containers: Reopen in Container` (first open pulls the image and runs `rosdep install`, ~1 min)
3. Once inside, open three integrated terminal tabs:

```bash
# Terminal 1 – simulator
ros2 run ca1_motor_ctrl drive_simulator_node

# Terminal 2 – drive controller
ros2 run ca1_motor_ctrl drive_controller_node

# Terminal 3 – web UI (optional, open http://localhost:8080)
ros2 run ca1_motor_ctrl web_ui
```

**Rebuilding after source changes** — source files are bind-mounted, so edits on the host are immediately visible inside the container. After any `.cpp` or `.hpp` change, rebuild with:

```bash
# Inside the container terminal, or Ctrl+Shift+B
colcon build --symlink-install --packages-select ca1_motor_ctrl
```

Then restart the affected node. With `--symlink-install`, non-compiled files (launch files, Python scripts) are live without a rebuild.

---

### Option B — Docker CLI (no VS Code)

Set a shell variable for the image name once:

```bash
export IMAGE=$(docker images --format '{{.Repository}}:{{.Tag}}' | grep 'vsc-ca1_motor_ctrl.*uid')
export SRC=/home/newton/ca1_motor_ctrl
```

**Build:**

```bash
docker run --rm \
  -v "$SRC":/home/newton/ros2_ws/src/ca1_motor_ctrl \
  -w /home/newton/ros2_ws/src/ca1_motor_ctrl \
  "$IMAGE" \
  bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select ca1_motor_ctrl"
```

Build artifacts land in `$SRC/build/` and `$SRC/install/` on the host, so they survive container restarts. Re-run this command after any source change.

**Run** (two terminals, `--net=host` required for DDS discovery):

```bash
# Terminal 1 – simulator
docker run --rm -it --net=host \
  -v "$SRC":/home/newton/ros2_ws/src/ca1_motor_ctrl \
  -e ROS_DOMAIN_ID=42 "$IMAGE" \
  bash -c "source /opt/ros/humble/setup.bash && source /home/newton/ros2_ws/src/ca1_motor_ctrl/install/setup.bash && ros2 run ca1_motor_ctrl drive_simulator_node"

# Terminal 2 – drive controller
docker run --rm -it --net=host \
  -v "$SRC":/home/newton/ros2_ws/src/ca1_motor_ctrl \
  -e ROS_DOMAIN_ID=42 "$IMAGE" \
  bash -c "source /opt/ros/humble/setup.bash && source /home/newton/ros2_ws/src/ca1_motor_ctrl/install/setup.bash && ros2 run ca1_motor_ctrl drive_controller_node"

# Terminal 3 – web UI (optional, open http://localhost:8080)
docker run --rm -it --net=host -p 8080:8080 \
  -v "$SRC":/home/newton/ros2_ws/src/ca1_motor_ctrl \
  -e ROS_DOMAIN_ID=42 "$IMAGE" \
  bash -c "source /opt/ros/humble/setup.bash && source /home/newton/ros2_ws/src/ca1_motor_ctrl/install/setup.bash && ros2 run ca1_motor_ctrl web_ui"
```

---

### Tests (inside the container or via docker CLI)

```bash
colcon test --packages-select ca1_motor_ctrl
colcon test-result --verbose
```

---

### Manual OD access for debugging

```bash
# Read statusword
ros2 service call /ca1/od_read ca1_motor_ctrl/srv/OdRead "{index: 0x6041, subindex: 0}"

# Write controlword (Shutdown command)
ros2 service call /ca1/od_write ca1_motor_ctrl/srv/OdWrite "{index: 0x6040, subindex: 0, value: 6}"
```

---

## CI

| Workflow | Trigger | What it does |
|---|---|---|
| [`ci.yml`](.github/workflows/ci.yml) | Every push / PR | Builds the package and runs all unit tests |
| [`docker-publish.yml`](.github/workflows/docker-publish.yml) | Push to `main` (Dockerfile changed) + weekly | Builds and pushes the dev image to `ghcr.io/bwoerfel/ca1_motor_ctrl:latest` |

The weekly rebuild picks up security patches from the `osrf/ros:humble-desktop` base image automatically.

---

## Object dictionary reference

| Index  | Sub | Name                       | Access | Default |
|--------|-----|----------------------------|--------|---------|
| 0x6040 |  0  | Controlword                | RW     | 0       |
| 0x6041 |  0  | Statusword                 | RO     | 0       |
| 0x6060 |  0  | Modes of operation         | RW     | 3 (PV)  |
| 0x6061 |  0  | Modes of operation display | RO     | 3 (PV)  |
| 0x60FF |  0  | Target velocity            | RW     | 0       |
| 0x606C |  0  | Velocity actual value      | RO     | 0       |
| 0x6064 |  0  | Position actual value      | RO     | 0       |
| 0x607A |  0  | Target position            | RW     | 0       |
| 0x6080 |  0  | Max motor speed            | RW     | 250     |
| 0x6083 |  0  | Profile acceleration       | RW     | 1000    |
| 0x6084 |  0  | Profile deceleration       | RW     | 1000    |
| 0x603F |  0  | Error code                 | RO     | 0       |

---

## CiA 402 state machine

```
                    +-----------------------+
   (power-on) ----> |  Switch On Disabled   | <----+----+
                    +----------+------------+      |    |
                               | Shutdown (0x06)   |    |
                               v                   |    |
                    +-----------------------+       |    |
                    |  Ready to Switch On   |       |    |
                    +----------+------------+       |    |
                               | Switch On (0x07)  |    |
                               v                   |    |
                    +-----------------------+       |    |
                    |      Switched On      |       |    | Fault Reset
                    +----------+------------+       |    | (bit 7 rising edge)
                               | Enable Operation  |    |
                               |       (0x0F)      |    |
                               v                   |    |
                    +-----------------------+       |  +-+-----+
                    |  Operation Enabled    | fault |  | Fault |
                    |   <- motor runs ->    | ------+->+-------+
                    +-----------+-----------+       |
                                | Quick Stop (0x02) |
                                v                   |
                    +-----------------------+        |
                    |  Quick Stop Active    | -------+
                    +-----------------------+  Enable Operation (0x0F)
```

---

## Test status

All tests pass on `main`.  The CI suite covers the simulator library only (unit
tests via GTest); the controller and web UI are exercised by running the nodes
manually.

| Test group              | Status |
|-------------------------|--------|
| OD read/write           | PASS   |
| Error handling          | PASS   |
| Initial state           | PASS   |
| State machine           | PASS   |
| Physics update          | PASS   |
| Reset                   | PASS   |
| Controller bring-up FSM | PASS   |
| Controller fault recovery | PASS |
