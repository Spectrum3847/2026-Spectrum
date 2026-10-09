# Commits and Pull Requests

*Audience: Reference. No prerequisites.*

How we use Git on this repo. Short version: small commits, descriptive messages, PRs reviewed before merging to `main`.

## Keep your branch current

Before opening a PR, and ideally every day or two while a feature is in flight, pull `main` into your branch. Drift is cheap to resolve in small bites and expensive once it is days behind.

```sh
git fetch origin
git merge origin/main          # or: git rebase origin/main for a flatter history
```

Both work. Merge keeps the record of when you synced; rebase rewrites your local commits on top of `main` so the eventual PR diff is clean. Do not rebase a branch that someone else is working from. They will get the rewritten history on their next pull, and that is a mess to recover from.

GUI users: GitHub Desktop's *Branch*, then *Update from main*, does the merge variant.

One-command version (fetch, fast-forward your branch, merge `origin/main`, stash and restore any local edits, print what came in): `bash .agents/skills/git-sync-upstream/scripts/sync_upstream.sh`, with `--dry-run` to preview. It never pushes.

## Commits

Each commit should answer *why* a change happened. The what is in the diff. Useful commit subjects on this repo read like:

* Add launcher RPM telemetry for distance tuning
* Tighten MT1 ambiguity threshold to 0.5 to drop bad poses
* Fix swerve module 3 offset after collision at FNC week 4

Subjects to avoid: `update`, `wip`, `fixes`, `changes from review`. Those tell future-you nothing.

Format:

* **First line 72 characters or fewer**, imperative mood (`Add`, `Fix`, `Refactor`), no trailing period.
* Blank line.
* Body paragraphs when the change is non-trivial: the rationale, the alternatives you considered, any follow-up.

Do not commit `BuildConstants.java` or formatter-only churn as standalone commits unless that is all the change is. Roll them into the substantive commit they belong to.

### Who You're Committing As

On a team-owned laptop, five people use the same checkout in one build session. A commit's author address is published on the PR ref permanently, so every commit carries one identity, set globally once per machine rather than per clone:

```sh
git config --global user.email "you@users.noreply.github.com"
git config --global user.name "Your Name"
```

Whatever local identity a clone has picked up belongs to whoever sat down last, so check before you commit that it's you:

```sh
git config user.name
```

A clone that picked up a previous user's local identity does not inherit the global one. Fix it with `git config user.email "$(git config --global user.email)"`. If the global lookup comes back empty, stop and ask for the address. Do not derive one from your name, your domains, or the address the harness happens to identify you by.

VS Code users: the Git Config User Profiles extension (see the README) switches this from the status bar. For a one-off commit under your own name without touching the machine's config:

```sh
git -c user.name="Your Name" -c user.email="id+you@users.noreply.github.com" commit -m "..."
```

The email has to be one GitHub recognizes as yours or the commit shows up authored by nobody. The `@users.noreply.github.com` form always works and doesn't publish your real address; find yours under **GitHub → Settings → Emails**.

AI agent sessions follow the same rule, enforced rather than trusted: see [Writing an AGENTS.md for a Robot Repo](../other-guides/agents-md-guidelines.md). Agent-generated commits are authored by the student driving the session, with no bot co-author trailer.

## Pull Requests

Open the PR against `main`. Before you click *Create*:

* **One feature per PR.** This is the rule we bend least. The point is removability: when a mechanism misbehaves at an event, backing out auto-aim should be one `git revert` of one squashed merge, not an hour of separating auto-aim from the LED cleanup that rode along with it. If you found an unrelated bug mid-branch, fix it on its own branch off `main`, even if it's one line. Refactors go in their own PR, separate from the behavior change they enable. If your description needs the word "also," you have two PRs.
* **CI must pass.** A PR with a red build doesn't get reviewed. Run `./gradlew clean build` locally first; if you can't be bothered locally, CI will catch it and you'll just iterate slower.
* **Pull `main` first.** A PR that doesn't merge cleanly is a PR that's hard to review.
* **Self-review the diff.** Skim it before assigning a reviewer. You'll catch debug prints, commented-out code, and accidental file moves about half the time.
* **Description**: one or two sentences on what the PR does and why. If there's a known limitation (e.g., "doesn't handle the X edge case yet, tracked in #123") call it out so reviewers don't waste time on it.

### Reviewers

Tag a programming lead, or whoever you collaborated with on the feature. Anything that touches `Robot.java`, `SuperStructure.java`, the swerve, or vision integration gets a second pair of eyes, because those files have the most blast radius.

### Merging

* The build must be green.
* All review comments resolved, either addressed or replied to with a reason.
* Use *Squash and merge* by default; it keeps the `main` history readable. Use *Rebase and merge* for a stack of intentionally separate commits, which is rare.
* **Never merge a red branch.** A broken `main` means everyone on the team is deploying broken code. Ten minutes to fix it locally is much cheaper than the next person's ninety minutes of asking why their unrelated feature stopped working.

## After a merge

Delete the branch; GitHub will offer. If the change included documentation, open `index.md` and click through to the pages you touched, because a broken relative link is invisible until somebody clicks it. See [Documentation and Comments](documentation-and-comments.md) for what to write up when.

## When things go sideways

A merge conflict you cannot resolve: push your branch, ping the reviewer in the PR, and resolve it together. Do not `git reset --hard` to start fresh without backing the work up first. Local-only commits that get reset are gone.

`git push --force` and `--force-with-lease` are appropriate after a rebase of *your own* branch, and never on a shared branch and never on `main`. If you are unsure, ask before force-pushing.
