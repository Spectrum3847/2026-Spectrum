---
name: git-sync-upstream
description: "Use when the user wants the current branch brought up to date with GitHub FAST: fetch origin, pull (fast-forward) the current branch, and merge origin/main (or a named branch) into it in one command, with local edits auto-stashed and restored, a conflict list if it stops, and a summary of what came in. Also for 'sync', 'update my branch', 'pull main in', 'get the latest'."
metadata:
  short-description: Fetch, pull, and merge origin/main into the current branch in one command
---

# Git Sync Upstream

One command that does what `docs/coding-conventions/commits-pull-requests.md` asks for under
"Keep Your Branch Current": fetch `origin`, bring the current branch level with its remote,
merge `origin/main` into it, and say what changed. It is for the common case between
practice sessions and before a PR. Anything unusual (rebasing a shared branch, resolving a
big conflict, force pushes) is a conversation with the user, not this skill.

## Core Rules

- **Run the script, do not hand-roll the git commands.** It already handles the dirty tree,
  the diverged branch, the missing remote branch, and the conflict report. One call, a few
  seconds, mostly the fetch.
- **Dry-run first only when the user asks what would come in.** Otherwise run it for real; a
  dry run costs a second fetch.
- **Never push, never deploy from this skill.** The merge of `origin/main` is a local commit.
  Report `ahead/behind` from the summary and let the user decide to push. `./gradlew deploy`
  is the user's call, always.
- **On exit code 2 (conflict), stop and report.** Paste the conflicted file list, say which
  step conflicted (pull vs. merge of main), and remind that local edits are still in the stash
  if the script said so. Do not resolve conflicts unasked; if asked, resolve, then
  `git add` + `git commit`, then `git stash pop` if there was an autostash.
- **On exit code 1 after a merge (stash restore conflicted), the merge itself succeeded.**
  The user's uncommitted edits collided with what came in. Report which files, and that the
  stash is kept until they run `git stash drop`.
- **Build after any real change.** If the summary says the branch moved, run `./gradlew build`
  (it also re-formats) before trusting the tree, and always when the build/vendordeps hint
  fires. Report the build result, not just "merged".
- **Pass the hints on.** The summary flags PathPlanner, robot config, state machine, docs,
  and skills changes because each one invalidates something an agent may have cached: auto
  names, CAN IDs, `Coordinator.java` structure, doc facts. Re-read before acting on them.
- **Read the dry run before merging `main` into a long-lived branch.** `main` may delete
  files this branch still relies on (it removed most of `.agents/skills/` on 2026-09-19).
  If the incoming commit list looks destructive, show it to the user before merging.

## Commands

Sync the current branch (fetch, fast-forward, merge `origin/main`, restore local edits):

```sh
bash .agents/skills/git-sync-upstream/scripts/sync_upstream.sh
```

Options:

- `--dry-run`: fetch, then list the commits the pull and the merge would bring in. Changes nothing.
- `--from <branch>`: merge `origin/<branch>` instead of `origin/main` (for example a feature
  branch this one is stacked on).
- `--no-merge-main`: only fetch and fast-forward the current branch.
- `--rebase`: rebase local commits onto `origin/<branch>` instead of merging when the branch has
  diverged. Only for the user's own unshared branch; the git conventions doc explains why.
- `--no-stash`: refuse to run on a dirty tree instead of autostashing.
- `--remote <name>`: another remote than `origin` (forks).

Exit codes: `0` synced or nothing to do, `1` precondition failed (merge or rebase already in
progress, unresolved conflicts, detached HEAD) or the stash restore conflicted after a
successful merge, `2` the pull or the merge stopped on a conflict.

## What the Output Means

```
== fetching origin (branch: 2026-offseason-bot)
fetch took 1 s

== stashing local edits
saved as: stash@{0}: On 2026-offseason-bot: sync_upstream autostash 2026-09-19T13:41:50

== fast-forwarding 2026-offseason-bot to origin/2026-offseason-bot (2 new commit(s))

== merging origin/main into 2026-offseason-bot (2 commit(s))

== restoring your local edits
local edits restored

== summary
2026-offseason-bot moved fe835d4a1 -> 9c0b2e7d3 (5 commit(s) in)
     47 files changed, 403 insertions(+), 4879 deletions(-)
  * docs or AGENTS.md changed: re-read the relevant page before acting on old assumptions
  * agent skills changed
run ./gradlew build before trusting the merged tree.
vs origin/2026-offseason-bot: 3 ahead, 0 behind. Nothing was pushed.
the merge of origin/main is a local commit; push only when asked.
```

- **"has diverged (N remote, M local): merging"**: local commits exist that are not on the
  remote, so a fast-forward was impossible and the script merged instead. Normal after
  committing locally while a teammate pushed. `--rebase` gives a flatter history if wanted.
- **"already contains origin/main"**: the merge step had nothing to do.
- **"no origin/<branch> on the remote"**: a local-only branch; only the merge step runs.
- **"could not restore your edits cleanly"**: the merge is done and committed; the uncommitted
  edits conflict with it. Files show as `UU` in `git status`. Resolve, `git add`, then
  `git stash drop`.

## Report Format

One message: what moved (commit count, short stat), the hints that fired, the build result,
and the ahead/behind line. If it stopped on a conflict, lead with the conflicted files and
the next command. Do not paste the whole script output.
