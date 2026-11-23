# Development shortcuts

*Audience: reference. Assumes you have read [Setup](../setup.md).*

Shortcuts and commands that come up often. If you are new, learn `F2`, `Ctrl+D`, `Ctrl+Shift+P` and the multi-cursor shortcuts first. They pay off right away.

## VS Code

| Shortcut | What it does |
|---|---|
| `Tab` | Accept an inline suggestion, if you use a code completion extension. |
| `F2` | Rename a variable, function or type everywhere it is used. Use this instead of find and replace. It knows scope. |
| `Ctrl+/` | Toggle a line comment on the selected text. |
| `Ctrl+Click` or `F12` | Go to definition. |
| `Alt+Shift+F12` | Find all references to a name. |
| `Ctrl+P` | Quick open. Search files by name. |
| `Ctrl+Shift+P` | Command Palette. Run any VS Code command by name. |
| `Ctrl+Shift+V` | Preview Markdown in a side panel. Useful when you edit docs. |
| `Shift+Alt+Down` | Duplicate the current line or block. |

## Multi-cursor editing

Multi-cursor lets you edit several places at once without regular expressions.

| Shortcut | What it does |
|---|---|
| `Ctrl+D` | Select the next copy of the highlighted text and add a cursor there. Press again for more. |
| `Ctrl+U` | Undo the last `Ctrl+D`. |
| `Ctrl+Shift+L` | Select every copy of the highlighted text at once. |
| `Ctrl+Alt+Up` or `Down` | Add a cursor above or below. |
| `Alt+Click` | Add a cursor where you click. |
| Middle click and drag | Column select. |
| `Alt+Shift+Arrow keys` | Expand or shrink the selection. |

## rust-analyzer commands

Run these from the Command Palette (`Ctrl+Shift+P`).

| Command | What it does |
|---|---|
| `rust-analyzer: Restart server` | Clear stale errors after a branch switch. |
| `rust-analyzer: Run` | Run a `main` function, test or example. |
| `rust-analyzer: Expand macro recursively` | Show what a macro writes. |

## Terminal commands

Run these from `rust/`. [Cargo xtask](../tools/cargo-xtask.md) has the full list.

```text
cargo xtask gate                    # format check, clippy, all tests
cargo fmt --all              # format the code
cargo xtask sim              # run the robot in simulation
cargo xtask build-roborio    # build for the roboRIO
cargo xtask deploy           # copy to the roboRIO
```

## Git in VS Code

The Source Control panel (`Ctrl+Shift+G`) handles staging, committing, pushing and pulling. The GitHub Pull Requests and Issues extension lets you review and comment on pull requests in the editor.

If you prefer the terminal, see [Commits and pull requests](../coding-conventions/commits-pull-requests.md) for our rules.

## Phoenix Tuner X

Phoenix Tuner X is a desktop app, not a VS Code extension, so it has no editor shortcuts. Its self-test snapshot, plotter and log export are in [Phoenix Tuner X](../tools/phoenix-tuner-x.md).
