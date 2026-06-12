# Development Shortcuts

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

A reference for the shortcuts and commands that come up often. If you're new, the ones worth memorizing first are `F2`, `Ctrl+D`, `Ctrl+Shift+P`, and the multi-cursor shortcuts — they pay off immediately.

## VS Code

| Shortcut | What it does |
| --- | --- |
| `Tab` | Accept an inline Copilot suggestion. |
| `Ctrl+Right Arrow` | Accept the next word of a suggestion. |
| `F2` | Rename — renames all references to a variable, method, or class across the project. Use this instead of find-and-replace; it's scope-aware. |
| `Ctrl+/` | Toggle line comment on the selected text. |
| `Ctrl+Click` or `F12` | Go to definition — jumps to where a method or class is declared. |
| `Alt+Shift+F12` | Find all references — shows every call site for a symbol. |
| `Ctrl+P` | Quick open — fuzzy-search files by name. Faster than the Explorer for navigating a codebase. |
| `Ctrl+Shift+P` | Command Palette — run any VS Code or WPILib command by name. This is how you launch sim, clean the workspace, or deploy. |
| `Ctrl+`` ` `` ` | Toggle the integrated terminal. |
| `Ctrl+Shift+V` | Preview Markdown in a side panel — useful when editing docs. |
| `Shift+Alt+Down` | Duplicate the current line or selected block. |

## Multi-Cursor Editing

Multi-cursor is one of the highest-value things to learn in VS Code. It lets you edit several places at once without regex.

| Shortcut | What it does |
| --- | --- |
| `Ctrl+D` | Select the next occurrence of the highlighted text and add a cursor there. Keep pressing to select more. |
| `Ctrl+U` | Undo the last `Ctrl+D` (deselect the most recently added cursor). |
| `Ctrl+Shift+L` | Select all occurrences of the highlighted text at once. |
| `Ctrl+Alt+Up/Down` | Add a cursor above or below the current line. |
| `Alt+Click` | Place a cursor at any clicked location. |
| `Middle Click + Drag` | Column select — places cursors on every line you drag across. |
| `Alt+Shift+Arrow Keys` | Expand or shrink the selection by word, line, or block. |

A common use: highlight `public`, then `Ctrl+Shift+L` to select every instance in a file, type `private` and you've renamed them all at once.

## WPILib Actions

These are all available through `Ctrl+Shift+P`. The most useful ones:

| Command | What it does |
| --- | --- |
| `WPILib: Simulate Robot Code` | Build and launch the simulator. Equivalent to `./gradlew simulateJava`. |
| `WPILib: Deploy Robot Code` | Build and deploy to a connected RoboRIO. Equivalent to `./gradlew deploy`. |
| `WPILib: Build Robot Code` | Compile without deploying. Equivalent to `./gradlew build`. |
| `Java: Clean Language Server Workspace` | Clear the Java language server cache. Use when IntelliSense is showing phantom errors. |
| `WPILib: Manage Vendor Libraries` | Add or update vendordep libraries (Phoenix 6, PathPlanner, etc.). |

Keyboard shortcuts for simulation and deploy (`F5`, `Shift+F5`) can be set up in VS Code's keybinding editor, but the Command Palette is reliable across any machine.

## Terminal Commands

For anything not in the WPILib menu, see [Gradle](../tools/gradle.md) for the full table of `./gradlew` tasks. The ones you'll run most often:

```
./gradlew build          # Compile + format + static analysis
./gradlew clean build    # Same, from scratch
./gradlew deploy         # Build and push to the RoboRIO
./gradlew spotlessApply  # Reformat all source files (run before committing if CI fails on format)
```

## Phoenix Tuner X

Phoenix Tuner X has a few actions that aren't in menus most people look at:

- **Self-Test Snapshot**: Run from the device page to dump firmware, config, faults, and temps in one shot. Save the snapshot before asking for help with a motor issue.
- **Plot tab**: Live-plot any signal pair. For PID tuning, add `Closed Loop Reference` and the matching `Position` or `Velocity` and hit Record. See [PID Tuning](../tools/pid-tuning.md) for how to interpret what you see.
- **Log Extractor**: Pull `.hoot` files off the CANivore after a match.

See [Phoenix Tuner X](../tools/phoenix-tuner-x.md) for the full workflow.

## Git in VS Code

The Source Control panel (`Ctrl+Shift+G`) handles staging, committing, pushing, and pulling. For branch management and pull requests, the GitHub Pull Requests and Issues extension integrates directly — you can review PRs, leave comments, and merge without leaving the editor.

If you prefer the terminal, see [Commits and Pull Requests](../coding-conventions/commits-pull-requests.md) for the conventions this project uses.
