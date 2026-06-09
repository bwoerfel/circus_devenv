# CA-1 Motor Controller – Code Review Setup

## Prerequisites (install once)

**macOS / Linux**
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) or Docker Engine
- [VS Code](https://code.visualstudio.com/) + the **Dev Containers** extension (`ms-vscode-remote.remote-containers`)

**Windows – option A: Docker Desktop**
- [Docker Desktop](https://www.docker.com/products/docker-desktop/) (enables WSL2 backend automatically)
- [VS Code](https://code.visualstudio.com/) + **Dev Containers** extension

**Windows – option B: WSL2 + native Docker Engine (no Docker Desktop)**
1. Enable WSL2 and install Ubuntu from the Microsoft Store
2. Inside the Ubuntu terminal, install Docker Engine:
   ```bash
   sudo apt-get update && sudo apt-get install -y docker.io
   sudo service docker start
   sudo usermod -aG docker $USER   # log out and back in after this
   ```
3. Install VS Code + the **WSL** extension (`ms-vscode-remote.remote-wsl`) + **Dev Containers** extension
4. Open the project from the Ubuntu terminal: `code .`
   VS Code connects to WSL2 and "Reopen in Container" works from there.

## Steps

```bash
git clone https://github.com/bwoerfel/circus_devenv.git
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
