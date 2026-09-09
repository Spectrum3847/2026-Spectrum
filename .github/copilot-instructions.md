# Copilot Instructions for 2026-Spectrum

**The repository guide is [`CLAUDE.md`](../CLAUDE.md) at the repo root. Read it first.**

It carries the build and test commands, the state-machine architecture, the
directory map, the task-routing table, and the coding conventions — for every
assistant, not just Claude. Keeping one file means one file to update when the
code moves. This page holds only what is specific to Copilot and its subagent
templates.

Until 2026-09 this file duplicated all of that guidance, and the copy drifted:
it described a `RobotStates` / `Coordinator` / `State.java` / `*States.java`
architecture that no longer exists in this tree, plus a vendored WPILib
`Trigger`, a `SpectrumSubsystem` interface, and `Zones` / `TagProperties` /
`HomeOffsets` classes that are not in the repo at all. Do not reintroduce a
second copy of the guide here.

---

## Subagents

Subagents are small, focused prompt templates used by Copilot-run subagents (or
via `runSubagent`) to perform repeatable repository tasks (searches,
scaffolding, small patches, audits). We keep templates as separate Markdown
files so they are discoverable and easily updated.

- Location: `.github/agents/`.
- Usage: When a user request maps to an existing template, prefer invoking that
  subagent. If no agent exists for a recurring task, propose creating one and
  ask before applying any changes.
- Adding a new subagent template:
  1. Add the new Markdown template to `.github/agents/` with YAML frontmatter
     including `name` and `description`.
  2. Add a one-line entry to `.github/agents/README.md` describing the template
     and example usage.
  3. Add a short bullet to the list in
     `.github/instructions/agent-usage.instructions.md`.
  4. Use `apply_patch` for edits and include a one-line rationale for each patch
     hunk. Do not edit generated files like
     `src/main/java/frc/robot/BuildConstants.java`.
- Permissions & behavior:
  - Copilot may create and edit subagent template files, but must never add
    secrets or sensitive data.
  - For code changes produced by subagents, produce `apply_patch` patches and do
    not automatically commit without human review.
  - Keep subagent templates minimal, with clear inputs/outputs and explicit
    instructions for `apply_patch` output when code changes are expected.

## Reminders for agents working in this repo

1. **Run `./gradlew build` after making Java changes** — Spotless auto-formats,
   and SpotBugs + ErrorProne will catch issues. If the build fails on
   formatting, re-run it.
2. **Do not manually edit `BuildConstants.java`** — it is regenerated on every
   build and is not committed.
3. **Line endings must be LF (UNIX).** Windows users: set
   `core.autocrlf=false` or `core.autocrlf=input`.
4. **When you learn something new about the repository, update
   [`CLAUDE.md`](../CLAUDE.md)** — not this file. Keep it factual and concise,
   and never add secrets or credentials.
