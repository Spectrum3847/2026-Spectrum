# Build Tools and Other Development Utilities

## Spotless

We started using Spotless to automatically format our code. This ensures consistent code style across the team, regardless of who writes the code. Spotless applies a standardized style whenever the project is built.

## SpotBugs

We are experimenting with SpotBugs to catch potential bugs early, before they are committed to our repository. It helps identify code that might not run or functions that are not calling the correct class/method.

*   Currently, it's configured for a low report level.
*   (April 2025) Found useful for spotting unreachable code or incorrect method calls.

## Lombok

We use Lombok annotations to simplify our coding by reducing boilerplate code.

*   **Annotations**: Lombok generates common methods like getters and setters using annotations (e.g., `@Getter`, `@Setter`).
*   **Reduced Code**: This significantly reduces the number of lines of code we have to write, making our codebase cleaner and more readable.
*   **Encapsulation**: It allows us to keep variables private while still easily providing public accessors.

## VS Code Extensions

These Visual Studio Code extensions enhance our development workflow:

*   **Language Support for Java™ by Red Hat** (`redhat.java`): Provides robust Java language features like IntelliSense, code navigation, and refactoring. You can also use "Clean the Java environment" if you encounter strange Java errors.
*   **Error Lens** (`usernamehw.errorlens`): Highlights errors and warnings directly in the code editor, making them easier to spot and fix.
*   **GitHub Copilot**: An AI pair programmer that provides auto-code completion suggestions.
*   **Copilot Chat**: Extends GitHub Copilot with chat capabilities for more interactive AI code assistance.
*   **GitHub Pull Requests and Issues** (`github.vscode-pull-request-github`): Allows you to view and manage GitHub Pull Requests and Issues directly within VS Code.
*   **GitLens** (`eamodio.gitlens`): Supercharges Git capabilities within VS Code, letting you see who committed changes, when, and why (e.g., via `git blame`).
*   **Git Config User Profiles**: Useful for managing multiple Git identities, allowing different people to commit from the same machine with their respective GitHub accounts. Works with `git blame`, etc.
*   **IntelliCode** (`VisualStudioExptTeam.vscodeintellicode`): Provides AI-assisted development features, including context-aware code completions.
*   **Spell Right** (`Ban.spellright`): A spell checker for VS Code.
*   **Open in Browser** (`techer.open-in-browser`): Useful for quickly opening HTML files (e.g., SpotBugs output) in a web browser.
*   **Live Share** (`ms-vsliveshare.vsliveshare`): Enables real-time collaborative development, allowing remote follow mode and pair programming. (We haven't used this much but would like to.)
