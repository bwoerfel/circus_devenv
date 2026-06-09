---
name: feedback_devcontainer_for_build
description: "Always use the devcontainer image (ghcr.io/bwoerfel/ca1_motor_ctrl) for building and testing, never the local host environment"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a742f69a-a230-4e95-bcc5-4118b983eaa7
---

All builds and tests must run inside the devcontainer image (`ghcr.io/bwoerfel/ca1_motor_ctrl:latest`), not the local host environment.

**Why:** The host may have a different ROS distro (e.g. lyrical vs humble) and different tool versions (uncrustify config, Python version, etc.) that produce different results and false lint passes. The devcontainer is the canonical environment matching CI.

**How to apply:**
- Never run `colcon build`, `colcon test`, or linters directly on the host to validate changes.
- Use `gh run view --log-failed` to inspect CI results (which run in the devcontainer image).
- If local testing is needed, do it inside the devcontainer: `Reopen in Container` in VS Code, or `docker run ghcr.io/bwoerfel/ca1_motor_ctrl:latest`.
- See [[feedback_use_gh_for_ci]] for how to check CI outcomes.
