---
name: feedback_maintain_docs
description: "Before every commit, update all documentation and verify the Sphinx/Breathe/Exhale build passes cleanly"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a742f69a-a230-4e95-bcc5-4118b983eaa7
---

Before every commit, update all affected documentation and verify the full docs pipeline builds without errors.

**Why:** Documentation drifts silently — new files, renamed nodes, added topics, changed commands all invalidate existing docs without any error. The user wants living documentation maintained alongside the code, including the Sphinx-based API reference.

**How to apply:**
- When adding a new node, topic, service, or file: update README (package layout, run instructions, OD reference) and ARCHITECTURE.md in the same commit.
- When changing a command (node name, CLI flags, ros2 run target): find and update every place it appears in README, ARCHITECTURE.md, and CLAUDE.md.
- When changing C++ headers (new classes, methods, constants): run `./build_docs.sh` to confirm Doxygen parses them and Sphinx builds cleanly (0 warnings, 0 errors).
- When changing prerequisites or setup steps: update the Prerequisites / Setup sections in README.
- Scan README, ARCHITECTURE.md, and CLAUDE.md headings after each change and ask: "is this section still accurate?" — layout table, build/run commands, CI table, OD reference, and node reference are the most likely to drift.
- Documentation updates must be part of the same commit as the code change, not a follow-up.

**Docs build command:** `./build_docs.sh` (runs Doxygen → Sphinx → Furo HTML; must exit 0 before committing)
