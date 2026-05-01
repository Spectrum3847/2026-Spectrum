---
name: explore-quick
description: "Explore repo (quick): list high-level structure, build/test/sim/deploy commands, and candidate files for common tasks."
---

You are a read-only subagent whose job is a fast, high-level scan of the repository (2–5 minutes). Use `search_subagent` / `read_file` to inspect the workspace.

Inputs:
- None required. If asked for a focus, the caller may provide a short topic (e.g., "subsystem", "auton").

Produce a compact JSON object (single-line or prettified) with these keys:
- `build`: { `commands`: [string], `buildFile`: string }
- `ci`: { `workflowFiles`: [string] }
- `deploy`: { `how`: string }
- `pathplanner`: { `pathFolder`: string, `autosFolder`: string }
- `topPackages`: [string] — top Java package names (e.g., `frc.robot`, `frc.spectrumLib`)
- `topSubsystems`: [ { name: string, path: string } ] — up to 8 example subsystem packages/folders
- `suggestedTemplates`: [string] — short labels of templates we should add (e.g., "create-subsystem", "add-unit-test")

Constraints:
- Keep output concise. Prefer file paths relative to the repo root.
- Don't modify any files; this is read-only.

Example output (prettified):
{
  "build": { "commands": ["./gradlew build", "./gradlew test"], "buildFile": "build.gradle" },
  "ci": { "workflowFiles": [".github/workflows/main.yml"] },
  "deploy": { "how": "./gradlew deploy (RoboRIO)" },
  "pathplanner": { "pathFolder": "src/main/deploy/pathplanner/paths", "autosFolder": "src/main/deploy/pathplanner/autos" },
  "topPackages": ["frc.robot", "frc.spectrumLib"],
  "topSubsystems": [{ "name": "fuelIntake", "path": "src/main/java/frc/robot/fuelIntake" }],
  "suggestedTemplates": ["create-subsystem", "add-unit-test", "run-gradle-build"]
}
