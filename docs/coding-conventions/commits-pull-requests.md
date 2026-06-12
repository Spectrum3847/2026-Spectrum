# Commits and Pull Requests

*Audience: Reference. No prerequisites.*

How we use Git on this repo. Short version: small commits, descriptive messages, PRs reviewed before merging to `main`.

## Keep Your Branch Current

Before opening a PR — and ideally every day or two while a feature is in flight — pull `main` into your branch. Drift is cheap to resolve in small bites and expensive once it's days behind.

```sh
git fetch origin
git merge origin/main          # or: git rebase origin/main for a flatter history
```

Both work. Merge keeps the history of when you synced; rebase rewrites your local commits on top of `main` so the eventual PR diff is clean. Don't rebase a branch that someone else is working from — they'll get the rewritten history on their next pull and it's a mess to recover from.

GUI users: GitHub Desktop's *Branch → Update from `main`* does the merge variant.

## Commits

Each commit should answer "*why*" a change happened. The "what" is in the diff. Examples of useful commit subjects on this repo:

* `Add launcher RPM telemetry for distance tuning`
* `Tighten MT1 ambiguity threshold to 0.5 to drop bad poses`
* `Fix swerve module 3 offset after collision at FNC week 4`

Examples of subjects to avoid: `update`, `wip`, `fixes`, `changes from review`. Those tell future-you nothing.

Format:

* **First line ≤ 72 chars**, imperative mood (`Add`, `Fix`, `Refactor`), no trailing period.
* Blank line.
* Body paragraph(s) when the change is non-trivial — explain rationale, alternatives considered, follow-up TODOs.

Don't commit `BuildConstants.java` changes or formatter-only churn as standalone commits unless that's all the PR is. Roll them into the substantive commit they go with.

## Pull Requests

Open the PR against `main`. Before clicking *Create*:

* **CI must pass.** A PR with a red build doesn't get reviewed. Run `./gradlew clean build` locally first — if you can't be bothered locally, CI will catch it and you'll just iterate slower.
* **Pull `main` first.** A PR that doesn't merge cleanly is a PR that's hard to review.
* **Self-review the diff.** Skim it before assigning a reviewer. You'll catch debug prints, commented-out code, and accidental file moves about half the time.
* **Description**: one or two sentences on what the PR does and why. If there's a known limitation (e.g., "doesn't handle the X edge case yet, tracked in #123") call it out so reviewers don't waste time on it.

### Reviewers

Tag a programming lead, or whoever you collaborated with on the feature. For anything that touches `Robot.java`, `Coordinator.java`, swerve, or vision integration, get a second pair of eyes — those files have the most blast radius.

### Merging

* Build must be green.
* All review comments resolved (either addressed or replied to with a reason).
* Use *Squash and merge* by default — keeps `main` history readable. Use *Rebase and merge* for a stack of intentionally separate commits (rare).
* **Never merge a red branch.** A broken `main` means everyone on the team is deploying broken code. The 10 minutes to fix it locally is much cheaper than the next person's 90 minutes of debugging "why doesn't my unrelated feature work."

## After a Merge

Delete the branch (GitHub will offer). If your feature came with documentation changes, double-check the rendered docs (open `index.md` and click through) to make sure links resolve. See [Documentation and Comments](documentation-and-comments.md) for what to write up when.

## When Things Go Sideways

Merge conflict you can't resolve? Push your branch, ping the reviewer in the PR, and resolve together. Don't `git reset --hard` to "start fresh" without backing the work up first — local-only commits that get reset are gone.

`git push --force` (and `--force-with-lease`) are appropriate after a rebase of *your own* branch, but never on shared branches and never on `main`. If you're unsure, ask before force-pushing.
