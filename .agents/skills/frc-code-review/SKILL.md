---

name: frc-code-review
description: >
Structured code review for FRC robot Java code. Use when asked to review, audit, or critique robot
code, or when checking code before competition. Covers safety, WPILib best practices, performance,
robustness, and competition readiness. Triggers on: "review my robot code", "check this subsystem",
"is this code competition-ready", or "what's wrong with this command".
----------------------------------------------------------------------

# FRC Code Review

## Overview

Review FRC robot Java code with a structured checklist before it ships to the field. Cover the promised areas:

- **Safety** — limit switches and hard stops, current/voltage limits, coast-vs-brake neutral mode, brownout behavior, and what happens on disable.
- **WPILib best practices** — scheduler-driven commands and triggers instead of imperative `periodic()` logic, requirement hygiene, correct `runsWhenDisabled` usage.
- **Performance** — CAN bus usage (signal rates, `optimizeBusUtilization`), loop-time impact, no blocking calls or allocations in hot paths.
- **Robustness** — attach/`isAttached()` guards for optional hardware, error handling for config statuses, crash recovery.
- **Competition readiness** — state-machine coverage, autonomous reliability, driver-station feedback, and what breaks if a sensor or motor dies mid-match.

Return findings as concrete `file:line` references with severity, a proposed fix, and whether the fix is required before competition.
