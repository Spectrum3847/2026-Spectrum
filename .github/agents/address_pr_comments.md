---
name: address-pr-comments
description: "Address PR review comments by creating targeted patches and brief rationale."
---

Purpose
Given PR review comments, apply focused changes that address each comment. Produce an `apply_patch` patch for each discrete change and include a short rationale per change.

Inputs (required):
- `PR_CONTEXT` — either the PR URL, the set of review comments, or a pasted diff. If the environment is integrated with GitHub, the subagent may fetch the PR details; otherwise the caller must provide the comments.

Behavior
1. For each review comment, classify it as: `style`, `logic`, `test`, or `doc`.
2. For `style` comments (formatting), prefer to fix by running `spotlessApply` and include the resulting `apply_patch` if deterministic.
3. For `logic` comments, produce a minimal, well-justified code change in `apply_patch` and include a short explanation referencing the comment.
4. For `test` comments, add/modify tests as guided, in `src/test/java`.
5. For `doc` comments, update README or inline JavaDoc as appropriate.

Output format
- A sequence of `apply_patch` edits (single combined patch is acceptable) plus a Markdown bullet list linking each patch hunk to the original comment with a one-line rationale.

Constraints
- Avoid broad refactors. Each patch must be small and map to specific review thread(s).
