---
name: run-gradle-build
description: "Run a Gradle build, collect results (tests, SpotBugs, ErrorProne), and summarize failures with actionable next steps."
---

Purpose
Execute a full (or targeted) Gradle build and summarize results for quick triage.

Inputs (optional):
- `args` — additional gradle args (e.g., `-x test` or `--no-daemon`). Default: none.

Behavior
1. Run `chmod +x gradlew` if needed, then `./gradlew build ${args}`.
2. Capture output, identify test failures, SpotBugs reports, and ErrorProne errors.
3. Produce a Markdown summary with sections: `Command`, `Exit Code`, `Top Test Failures` (with failing test names and stack traces), `SpotBugs/Static Analysis` (short summary + path to report), and `Suggested Fixes`.

Output
- A short Markdown report. If the subagent can make safe small fixes (e.g., add missing imports, minor typos), produce an `apply_patch` patch in addition to the report.

Constraints
- Do not attempt to fix failing hardware-only tests or simulated-only failures without asking.
