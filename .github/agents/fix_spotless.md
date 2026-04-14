---
name: fix-spotless
description: "Run Spotless formatting and Gradle build; if build errors are formatting-related, produce fixes as apply_patch."
---

Purpose
Run the project's formatting (Spotless) and build, then either apply automatic fixes or produce a concrete `apply_patch` with minimal changes to resolve build/format failures.

Behavior
1. Run `./gradlew spotlessApply`.
2. Run `./gradlew build`.
3. If the build succeeds, return a short summary: which commands were run and their status.
4. If the build fails and the failures are small (formatting, missing imports, simple typos), produce an `apply_patch` patch that fixes them. Otherwise, produce a short diagnostics report explaining next steps.

Output
- If no code changes needed: a short Markdown report with commands executed and success status.
- If code changes needed: produce an `apply_patch`-style patch and a one-paragraph justification for each change.

Permissions
- This template expects the subagent to be allowed to run `run_in_terminal` and `apply_patch`.

Constraints
- Prefer `spotlessApply` first; do not edit generated files like `BuildConstants.java`.
