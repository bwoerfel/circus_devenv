# CA-1 Motor Controller – Code Review Setup

## Prerequisites (install once)

- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (Windows / macOS) or Docker Engine (Linux)
- [VS Code](https://code.visualstudio.com/) + the **Dev Containers** extension (`ms-vscode-remote.remote-containers`)

## Steps

```bash
git clone <repo-url>
```

Open the cloned folder in VS Code → a notification pops up asking *"Reopen in Container"* → click it.

VS Code pulls the pre-built image (~2 GB, one-time download), starts the container, and builds the workspace automatically. This takes 3–5 minutes on the first run.

## Verify everything works

Open the integrated terminal and run:

```bash
colcon test --packages-select ca1_motor_ctrl
colcon test-result --verbose
```

All 54 tests should pass.

## Run the stack

```bash
# Terminal 1 – simulator
ros2 run ca1_motor_ctrl drive_simulator_node

# Terminal 2 – controller
ros2 run ca1_motor_ctrl drive_controller_node

# Terminal 3 – web UI  (open http://localhost:5000)
ros2 run ca1_motor_ctrl web_ui
```
