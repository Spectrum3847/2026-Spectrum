# Subagent templates

This folder contains ready-to-use prompt templates (subagent prompts) intended for use with `runSubagent` or as input to Copilot-run subagents. Each template is a focused instruction with a small frontmatter for discovery (`name` and `description`) and a clear output format.

Usage
 - Load the template content and call `runSubagent` with it as the `prompt` and a short `description`.

Example (assistant-side):

```
const prompt = fs.readFileSync('.github/subagents/templates/create_subsystem.md', 'utf8')
runSubagent({ prompt, description: 'Create subsystem (template)' })
```

Templates included
 - `.github/subagents/templates/explore_quick.md` — Quick repo scan: build/test/sim/deploy, top packages
 - `.github/subagents/templates/explore_thorough.md` — Thorough repo audit and recommended template list
 - `.github/subagents/templates/create_subsystem.md` — Create a subsystem skeleton (Mechanism + States + registration)
 - `.github/subagents/templates/add_unit_test.md` — Generate a JUnit 5 test skeleton for a target class
 - `.github/subagents/templates/fix_spotless.md` — Run Spotless & Gradle and produce a fix plan / patches
 - `.github/subagents/templates/run_gradle_build.md` — Run a Gradle build and summarize results (tests, SpotBugs, ErrorProne)
 - `.github/subagents/templates/address_pr_comments.md` — Address PR review comments with focused patches

Notes
 - Each template contains placeholders like `{{SUBSYSTEM_NAME}}` or `{{TARGET_CLASS}}`. Replace them or pass them to the subagent as parameters in your prompt.
 - Templates expect the subagent to output either (a) a human-readable plan, or (b) an `apply_patch`-style patch ready to be applied. Prefer `apply_patch` format when making repo changes.

Want more templates? Tell me which tasks to prioritize (e.g., pathplanner autos, swerve tuning, simulation harnesses).
