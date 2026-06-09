---
name: feedback_use_gh_for_ci
description: Always use gh CLI to check CI status and failures instead of waiting for user to paste logs
metadata: 
  node_type: memory
  type: feedback
  originSessionId: a742f69a-a230-4e95-bcc5-4118b983eaa7
---

Use `gh` CLI for CI debugging — check run status, fetch logs, and diagnose failures directly rather than asking the user to paste output.

**Why:** User explicitly requested this after manually pasting a CI failure log.

**How to apply:**
- After pushing, use `gh run list --repo bwoerfel/circus_devenv` to check run status.
- To get failure details: `gh run view <run-id> --repo bwoerfel/circus_devenv --log-failed`
- Proactively check CI after every push instead of waiting for the user to report failures.
