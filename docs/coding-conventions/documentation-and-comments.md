# Documentation and Comments

Effective documentation and commenting are vital for team collaboration, code maintainability, and helping new programmers understand the codebase.

## General Documentation Principles

*   **Helps Fellow Programmers**: Documentation clarifies your code's purpose, logic, and design, making it easier for others to understand and review your work.
*   **Spectrum Code Guide**: If you discover a particularly helpful approach or insight, contribute it to the Spectrum Code Guide to benefit future team members.
*   **GitHub Issues**: Utilize GitHub Issues for tracking tasks, bugs, and enhancements. Documenting issues helps resolve problems faster if the context is known.
*   **External Resources**: Don't limit yourself to internal documentation. Consult resources like Chief Delphi forums or official WPILib documentation for solutions and best practices.

## JavaDoc Comments

"Every class and nontrivial public method that you write must contain a Javadoc comment with at least one sentence describing what the class or method does. This sentence should start with a third person descriptive verb."

*   Use `/** Comment */` syntax for JavaDoc comments.
*   **Purpose**: JavaDoc comments are used to generate API documentation automatically. They explain the purpose of classes, methods, and variables.
*   **Content**: Describe *what* the class/method does, its parameters, return values, and any exceptions it might throw.
*   **Tools**: GitHub Copilot can often assist in generating basic JavaDoc comments.
*   **Full Guide**: Refer to the official JavaDoc guide for comprehensive details.

## TODO Comments

Use `TODO` comments for code that is:

*   Temporary.
*   A short-term solution.
*   Functional but could be improved.

**Format**: `// TODO: Followed by a colon and a descriptive message.`
*   These comments often appear as tasks in the VS Code console, making them easy to track for future improvements.

## Design Slide Building

Building design review slides effectively relies heavily on clear and accessible code documentation. When preparing for design reviews (e.g., subsystem designs, architectural overviews, new feature proposals), your documentation serves as a critical resource.

*   **Understanding "Why"**: Good comments and general documentation (like this guide) explain the rationale behind design choices, making it easier to present the "why" to others.
*   **Code Structure Overview**: Class-level JavaDocs and module documentation help you quickly summarize the purpose and interfaces of different code components for your slides.
*   **Behavioral Details**: Method-level JavaDocs and descriptions of state transitions (e.g., in `Coordinator.java` or `RobotStates.java`) provide the necessary detail to explain how specific functionalities work.
*   **Debugging/Testing Insights**: Documentation around potential edge cases, known bugs (TODOs), or testing procedures can inform discussions about system robustness during reviews.
*   **Consistency**: Adhering to code style and documentation standards ensures that when you pull information from the codebase for slides, it is presented clearly and consistently.

