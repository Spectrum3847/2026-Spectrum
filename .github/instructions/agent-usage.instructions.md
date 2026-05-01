# Agent Usage Guidance

The agents folder contains ready-to-use agent prompts intended for use with `runSubagent` or as input to Copilot-run subagents. Each is a focused instruction with a small frontmatter for discovery (`name` and `description`) and a clear output format.

Usage
 - Load the files content and call `runSubagent` with it as the `prompt` and a short `description`.

Example (assistant-side):

```
const prompt = fs.readFileSync('.github/agents/create_subsystem.md', 'utf8')
runSubagent({ prompt, description: 'Create subsystem (template)' })
```

Agents included
 - `explore_quick.md` — Quick repo scan: build/test/sim/deploy, top packages
 - `explore_thorough.md` — Thorough repo audit and recommended template list
 - `create_subsystem.md` — Create a subsystem skeleton (Mechanism + States + registration)
 - `add_unit_test.md` — Generate a JUnit 5 test skeleton for a target class
 - `fix_spotless.md` — Run Spotless & Gradle and produce a fix plan / patches
 - `run_gradle_build.md` — Run a Gradle build and summarize results (tests, SpotBugs, ErrorProne)
 - `address_pr_comments.md` — Address PR review comments with focused patches
 - `add_robot_state.md` — Add a new high-level robot state and wire it into the state machine
 - `verification_copilot.md` — Verify changes with build/test evidence and a PASS/FAIL verdict
 - `general_purpose_copilot.md` — General repo search, small patch, and analysis guidance

See `.github/agents/README.md` for the current list of templates and usage examples.

Notes
 - Each agent contains placeholders like `{{SUBSYSTEM_NAME}}` or `{{TARGET_CLASS}}`. Replace them or pass them to the subagent as parameters in your prompt.
 - Agents expect the subagent to output either (a) a human-readable plan, or (b) an `apply_patch`-style patch ready to be applied. Prefer `apply_patch` format when making repo changes.
