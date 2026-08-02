---
name: coderabbitai-review
description: Review an open PR with CodeRabbit — triage its comments, fix valid issues, push, and loop until the review approves. Use when a PR on this repo has a CodeRabbit review pending or needs one.
license: MIT
---

# CodeRabbit PR Review Loop

CodeRabbit is an AI code-review bot that comments on PRs. This skill covers the full loop: read the review, triage each comment, fix what's valid, push, and repeat until CodeRabbit approves the PR.

## Prerequisites

- `gh` (GitHub CLI) authenticated, with access to the repo and PR.
- A local checkout on the PR branch with a clean or known working tree.

## Workflow

### 1. Find the PR

```sh
gh pr view --json number,title,url,state
```

If the branch is pushed and a PR exists, note the PR number and URL.

### 2. Read the CodeRabbit review

```sh
gh pr view <PR> --comments        # review bodies and bot comments
gh pr diff                        # the diff being reviewed
```

If no CodeRabbit review appears, check `gh pr checks <PR>` for the CodeRabbit
status check, and note the bot may still be generating its review.

### 3. Triage each comment

Classify every comment:

| Class | Action |
| --- | --- |
| Valid fix | Fix it in code/docs, commit, push. |
| False positive / out of scope | Reply to the thread with a short justification (cite file:line evidence) and resolve it. |
| Already fixed | Reply pointing at the commit that fixed it. |
| Style preference only | Apply only if it matches the repo conventions in `docs/coding-conventions/`; otherwise reply briefly and resolve. |

Rule of thumb: fix fast, cheap, real issues; reply with evidence for everything
else. Never blindly apply a suggestion — verify it against the actual code
first (read the file, check the behavior).

### 4. Fix and push

1. Make minimal edits.
2. Validate: `./gradlew build` (Spotless auto-formats; re-run if it fails on
   formatting).
3. Commit with a concise message (see `docs/coding-conventions/commits-pull-requests.md`).
4. Push: `git push`.

### 5. Loop

After pushing, wait for CodeRabbit's next review round (it re-reviews each push):

```sh
gh pr checks <PR> --watch          # wait for checks to settle
gh pr view <PR> --comments         # read the new review
```

Repeat steps 3–5 until CodeRabbit approves the PR.

### 6. When approved

- Leave the approval in place.
- Summarize for the user: what was reviewed, what was fixed, what was skipped
  and why, and the final check status.

## Notes Learned From Real Runs

- CodeRabbit comments on specific lines of the diff; `gh pr view --comments`
  shows them inline. Pushes that don't change the line may not re-trigger a
  comment — the bot reports "existing comments not addressed" instead.
- The bot runs as a status check; a green status can arrive before the review
  comment is posted, so re-check comments after checks pass.
- Keep replies on the thread rather than in the PR body so the bot's
  conversation stays in one place.

## Further Reading

- `docs/coding-conventions/commits-pull-requests.md` — commit and PR conventions
- `docs/tools/build-tools.md` — build and static-analysis tooling
