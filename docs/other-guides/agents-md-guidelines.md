# Writing an AGENTS.md for a Robot Repo

*Audience: Reference. Assumes you've read [Commits and Pull Requests](../coding-conventions/commits-pull-requests.md).*

`AGENTS.md` is the file an AI coding agent reads before it touches anything. Claude Code, opencode, Cursor, and Copilot all look for it (Claude Code reads `CLAUDE.md`, which in this repo is a single `@AGENTS.md` include so there's only one file to maintain). Treat it as onboarding for a programmer who is fast, has no memory of last season, and will do exactly what you wrote down and nothing you didn't.

Two rules govern everything below:

* **Short beats complete.** It's loaded into context on every session. A 1000-line file dilutes the 20 lines that matter. If it isn't something an agent gets wrong without being told, it belongs in `docs/`, not here.
* **Point, don't duplicate.** Say "vision behavior lives in `docs/tools/vision.md`" instead of re-explaining MegaTag. Duplicated facts rot, and a rotted `AGENTS.md` is worse than none.

## What Every Robot Repo Should Cover

### 1. Identity and stack, in three lines

Team, season, game, language version, and the big frameworks with versions. This is what stops an agent from writing 2024 WPILib command syntax into a 2026 repo.

> FRC Team Spectrum 3847 robot code for the 2026 season (REBUILT): Java 17, GradleRIO 2026.2.1, WPILib 2026.

### 2. Where the truth lives

One section that says "read `docs/` before guessing," with a map of which page answers which question. An agent that knows `docs/tools/vision.md` exists will read it; one that doesn't will invent a pose estimator.

### 3. Build, format, and test commands

The exact commands, and which one must pass before anything is considered done. Include the non-obvious parts: our build auto-formats, so the first build can "fail" on formatting and succeed on a re-run. An agent that doesn't know that will try to fix a nonexistent bug.

```sh
./gradlew build          # compile + Spotless + tests + SpotBugs
./gradlew simulateJava
./gradlew deploy
```

### 4. Files that must never be edited

Generated files (`BuildConstants.java`), vendored patches (our copy of WPILib's `Trigger`), vendordep JSON. Say *why* in half a sentence, or an agent will "fix" the patched file back to upstream.

### 5. An architecture map, not an architecture essay

Enough to route a change to the right file. For us that's: state machine flows `State.java` → `RobotStates.java` → `Coordinator.java`; subsystems are `X.java` + `XStates.java`; hardware constants live in the `*2026.java` config classes, never in subsystem files.

### 6. "If you add X, also touch Y" checklists

The single highest-value thing in the file, because it's the thing humans forget too:

* New subsystem → register in `Robot.java` and `Coordinator.java`.
* New state → `State.java` + `Coordinator.java` + `RobotStates.java`.
* New path/auto → `src/main/deploy/pathplanner/` + register in `Auton.java`.
* New CAN device → the robot config class, not the subsystem.

### 7. Robot-safety rules

This is the section generic `AGENTS.md` templates don't have and FRC repos need most. An agent will happily raise a current limit to "fix" a mechanism that won't move.

* Don't change current limits, soft limits, or gear ratios without a lead's sign-off; those are the constants that break hardware.
* Prefer simulation (`./gradlew simulateJava`) for verification. Never assume a change is "tested" because it compiles.
* Never `./gradlew deploy` on the agent's own initiative. Deploying is a human-with-eyes-on-the-robot action.
* Encoder offsets and CAN IDs are per-robot facts; changing one in the wrong config class silently breaks a different robot.

### 8. Competition-day posture

Worth adding before your first event: during an event, `main` is frozen to what's on the practice-field robot; changes are minimal, reviewed in person, and deployed by a human. An agent should know that "it's comp weekend" changes the rules.

### 9. Git and PR policy

See [Commit Identity](#commit-identity-on-shared-computers) and [One Feature Per PR](#one-feature-per-pr) below. Both belong in `AGENTS.md` itself, not only in `docs/`, because they govern actions the agent takes rather than code it writes.

### 10. Secrets and posting policy

No credentials, no API keys, no student personal information in the repo. If the agent can comment on GitHub or post to Slack, say when it may and may not.

### 11. A "keep this current" line

Ask the agent to append what it learns, with a source and a date, the way the entries in our `AGENTS.md` are marked `(Verified 2026-08-03: ...)`. A dated claim can be re-checked; an undated one gets trusted forever.

---

## Commit Identity on Shared Computers

The problem: a team laptop has one `git config user.name`, five students use it during a build session, and every commit lands under whichever name was set last. VS Code users already solve this with the Git Config User Profiles extension (see the README). Agent sessions need their own answer, because the agent runs `git commit` itself.

**Don't** set a global identity and hope. **Do** make an unattributed commit impossible and have the session ask.

### The mechanism

Three pieces, all in this repo under `.claude/`:

1. **`.claude/hooks/session-git-identity.sh`** runs on `SessionStart`. It prints the repo's current git identity into the session's context along with the rule: ask who is driving before the first commit.
2. **`.claude/hooks/require-committer.sh`** runs on `PreToolUse` for `Bash`. Any command that invokes `git ... commit` without an explicit `-c user.name=` and `-c user.email=` is denied and the agent is told to ask; so is one that supplies an agent identity. There is no path to an anonymous commit. `.claude/hooks/require-committer.test.sh` is its test matrix, including the false-positive cases (`git status && echo commit` has to be left alone); run it after editing the hook.
3. **`.claude/committers.json`** is the team roster: display name plus GitHub no-reply email. The agent reads it and offers the names as choices, so a student picks from a list instead of spelling their email.

**The ask is once per session, not once per commit.** The first commit gets blocked, the agent asks, and the rest of the session reuses that answer; the next session starts over, because `session-git-identity.sh` fires again with no memory of the last one. One student per session is the normal case, and nagging them before every commit would only teach them to tune it out. Worth knowing what this does and doesn't buy you: the hook can see that an identity is *present* on the command, but not whether the agent asked you or remembered from ten minutes ago. A literal per-commit re-ask can't be enforced here — it would only ever be an instruction the agent chooses to follow. Enforced-once beats trusted-every-time.

The resulting commit looks like this, and nothing on the machine is mutated, so the next student starts clean:

```sh
git -c user.name="Jane Doe" \
    -c user.email="12345678+janedoe@users.noreply.github.com" \
    commit -m "Tighten MT1 ambiguity threshold to 0.5"
```

### Filling in the roster

Each entry needs the GitHub no-reply address, which is the one GitHub links back to an account without publishing a real email. Find yours at **GitHub → Settings → Emails**; it has the form `<id>+<username>@users.noreply.github.com`. Any other address only shows as that account if it's a *verified* email on it; otherwise the commit shows a grey silhouette and counts toward nobody's contributions.

Add students to `.claude/committers.json` as they join. That file is the only thing that needs maintaining.

### Belt and suspenders

Set this on every team-owned laptop, once:

```sh
git config --global user.useConfigOnly true
git config --global --unset user.name
git config --global --unset user.email
```

`user.useConfigOnly` makes git refuse to guess an identity from the hostname and login. Command-line commits then fail loudly with "no name was given" instead of quietly landing under `robotics@Team3847-Laptop3.local`. The `-c` flags above still satisfy it.

This also covers what the hook deliberately doesn't. The hook watches `git commit`, which is the case that matters; the other commands that write history with your identity attached (`merge` creating a merge commit, `rebase`, `cherry-pick`, `revert`) are caught by `useConfigOnly` refusing to invent an author.

### Two caveats

* Hooks live in `.claude/settings.json`, which Claude Code only runs after you trust the project on that machine. On a fresh laptop, the first session asks; say yes once.
* The hook governs the agent, not the student. Someone can still commit by hand under a stale identity, which is what `useConfigOnly` and the VS Code profile extension are for. Use all three.

---

## No Agent Attribution

Two separate things add agent attribution, and both have to be turned off.

**The trailer and PR text.** Claude Code appends `Co-Authored-By:` to commits and a generated-with line to PR bodies. `.claude/settings.json` in this repo turns both off:

```json
{
  "attribution": {
    "commit": "",
    "pr": "",
    "sessionUrl": false
  }
}
```

(`includeCoAuthoredBy: false` is the older key for the same thing; it still works but `attribution` is what to write in new configs, and it's the one that also removes the session URL from PR descriptions.)

**The author field.** Attribution settings don't touch it: if the identity at commit time is a bot's, the commit is a bot's. The `-c user.name` flow above is what actually puts the student's account on the commit. Both halves are required — settings alone leave commits authored by `Claude <noreply@anthropic.com>`.

If you also want the agent's built-in git workflow instructions out of the way entirely, `"includeGitInstructions": false` removes them and leaves only what `AGENTS.md` says.

---

## One Feature Per PR

The reason is removability. Halfway through an event, "back out the turret auto-aim" needs to be one revert of one merge commit, not an archaeology session separating auto-aim from an LED refactor that rode along in the same PR.

The rules worth writing into `AGENTS.md`:

* **One branch, one feature, one PR.** Named for the feature: `turret-auto-aim`, not `fixes` or `allen-branch`.
* **Unrelated fix found mid-branch?** It goes on its own branch off `main` and gets its own PR. A one-line typo fix is not an exception; it's the cheapest possible separate PR.
* **Refactors ship separately from behavior changes.** A diff that both moves code and changes what it does is unreviewable and unrevertable.
* **Formatter churn doesn't ride along.** Spotless touching twelve unrelated files buries the four lines that matter.
* **Dependent features stack.** If B genuinely needs A, open A's PR first, branch B off A, and say so in B's description. Don't merge them into one PR to save a round trip.
* **Squash and merge**, so each feature is exactly one commit on `main`. That's what makes `git revert <sha>` a complete undo.
* **The PR description names the one feature.** If writing it requires the word "also," it's two PRs.

For an agent specifically: it should ask before adding anything to a branch that isn't the feature it was asked for, and it should never open a PR unless a human asked for one.

---

## A Starting Skeleton

For a new robot repo, in this order:

```markdown
# Agent Instructions

<team, season, game, language, framework versions>

## Documentation
<where the real docs are, and which page answers what>

## Build & Development
<commands; what must pass; formatter behavior; generated files>

## Architecture in Brief
<the map: state machine, subsystems, configs, autos, telemetry>

## Robot Safety
<current limits, deploy policy, sim-first, per-robot configs>

## Git and PRs
<commit identity, no agent attribution, one feature per PR, branch naming>

## Important Notes
<the "if you add X, also touch Y" checklists>
<keep this file current: append facts with source + date>
```

Then delete every line that isn't true for your repo. A rule an agent can't follow teaches it to skim the rest.
