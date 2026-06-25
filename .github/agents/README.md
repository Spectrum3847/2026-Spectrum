# Agent Templates

This folder contains Copilot/agent prompt templates used by `runSubagent`, `search_subagent`, or similar tools.

Available templates
- `address_pr_comments.md` — Address PR review comments with focused patches.
- `add_unit_test.md` — Generate a JUnit 5 test skeleton for a target class.
- `add_robot_state.md` — Add a new high-level robot state and wire it into `State.java`, `RobotStates.java`, and `Coordinator.java`.
- `create_subsystem.md` — Create a new subsystem skeleton following repository conventions.
- `explore_quick.md` — Quick repository scan for build/CI/auton support.
- `explore_thorough.md` — Thorough repository audit and recommended templates.
- `fix_spotless.md` — Run Spotless and Gradle build, producing a fix plan or patch.
- `general_purpose_copilot.md` — General repo research, analysis, and small patch guidance.
- `run_gradle_build.md` — Run a Gradle build and summarize failures.
- `verification_copilot.md` — Verification-only agent for build/test evidence and PASS/FAIL verdicts.

Usage

```js
const prompt = fs.readFileSync('.github/agents/<template>.md', 'utf8')
runSubagent({ prompt, description: '...' })
```

Notes
- All templates use YAML frontmatter with `name` and `description`.
- Prefer output in `apply_patch` format for code changes.
