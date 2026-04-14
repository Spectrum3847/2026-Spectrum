---
name: add-unit-test
description: "Generate a JUnit 5 test skeleton for a target class in src/test/java following project conventions."
---

Purpose
Create a JUnit 5 test file skeleton for `{{TARGET_CLASS}}`.

Inputs (required):
- `TARGET_CLASS` — fully qualified class name or relative path (e.g., `frc.robot.launcher.Launcher` or `src/main/java/frc/robot/launcher/Launcher.java`).

Behavior
1. Locate the target class file and determine its package.
2. Create a test file at `src/test/java/<same package>/<ClassName>Test.java`.
3. Add JUnit 5 imports and at least two placeholder tests (constructor/initialization and one public method) with `@Test` and `@DisplayName` annotations.
4. If the class has public static helper methods, add a placeholder test for one of them.

Output format: produce an `apply_patch`-style patch that adds the test file. Include clear TODO comments in the generated test methods indicating what to assert.

Constraints
- Use JUnit 5 and follow project package naming.
- Keep tests minimal and compilable, with placeholders for assertions.
