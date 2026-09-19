#!/usr/bin/env bash
# sync_upstream.sh - bring the current branch up to date with origin in one command.
#
#   1. git fetch --prune origin
#   2. fast-forward (or --rebase) the current branch onto origin/<branch>
#   3. merge origin/main (or --from <branch>) into the current branch
#   4. report what came in and whether ./gradlew build is needed
#
# Local edits are stashed before step 2 and restored at the end, so a dirty tree
# is fine. Nothing is ever pushed or deployed. Exit codes: 0 ok, 1 precondition
# or usage error, 2 merge conflict (the script stops and tells you what to do).
#
# Usage: sync_upstream.sh [--from <branch>] [--rebase] [--no-merge-main]
#                         [--no-stash] [--dry-run] [--remote <name>]

set -euo pipefail

REMOTE=origin
FROM=main
REBASE=0
MERGE_MAIN=1
STASH=1
DRY_RUN=0

usage() {
    sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-1}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --from) FROM="$2"; shift 2 ;;
        --remote) REMOTE="$2"; shift 2 ;;
        --rebase) REBASE=1; shift ;;
        --no-merge-main) MERGE_MAIN=0; shift ;;
        --no-stash) STASH=0; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h|--help) usage 0 ;;
        *) echo "unknown option: $1" >&2; usage 1 ;;
    esac
done

say()  { printf '\n== %s\n' "$*"; }
warn() { printf '!! %s\n' "$*" >&2; }
die()  { warn "$*"; exit 1; }

git rev-parse --is-inside-work-tree >/dev/null 2>&1 || die "not inside a git repository"
cd "$(git rev-parse --show-toplevel)"

git_dir=$(git rev-parse --git-dir)
if [ -e "$git_dir/MERGE_HEAD" ]; then
    die "a merge is already in progress. Finish it (git commit) or abort it (git merge --abort) first."
fi
if [ -d "$git_dir/rebase-merge" ] || [ -d "$git_dir/rebase-apply" ]; then
    die "a rebase is already in progress. Finish it (git rebase --continue) or abort it (git rebase --abort) first."
fi
if [ -n "$(git diff --name-only --diff-filter=U)" ]; then
    warn "unresolved conflicts in the working tree:"
    git diff --name-only --diff-filter=U | sed 's/^/     /' >&2
    die "resolve them (edit, then git add <files>), or drop them (git checkout -- <files>), then rerun. A leftover autostash shows in: git stash list"
fi

BRANCH=$(git symbolic-ref --short -q HEAD) || die "detached HEAD; check out a branch first."
START=$(git rev-parse HEAD)
STASHED=0

restore_stash() {
    if [ "$STASHED" -eq 1 ]; then
        say "restoring your local edits"
        if git stash pop --quiet; then
            echo "local edits restored"
        else
            warn "could not restore your edits cleanly; they are still saved in the stash."
            warn "resolve the files above, then: git stash drop"
            git stash list | head -3
            STASHED=2
        fi
    fi
}

conflict_stop() {
    # $1 = what we were doing
    printf '\n!! CONFLICT while %s. Conflicted files:\n' "$1" >&2
    git diff --name-only --diff-filter=U | sed 's/^/     /' >&2
    printf '\n' >&2
    printf '   Fix each file, then:  git add <files> && git commit      (merge)\n' >&2
    printf '                     or:  git add <files> && git rebase --continue\n' >&2
    printf '   To back out entirely:  git merge --abort   (or git rebase --abort)\n' >&2
    if [ "$STASHED" -eq 1 ]; then
        warn "your local edits are still stashed. After the conflict is resolved: git stash pop"
    fi
    exit 2
}

# ---------------------------------------------------------------- 1. fetch
say "fetching $REMOTE (branch: $BRANCH)"
t0=$(date +%s)
git fetch --prune "$REMOTE"
echo "fetch took $(( $(date +%s) - t0 )) s"

has_upstream=0
git rev-parse --verify -q "$REMOTE/$BRANCH" >/dev/null && has_upstream=1
has_from=0
git rev-parse --verify -q "$REMOTE/$FROM" >/dev/null && has_from=1

if [ "$DRY_RUN" -eq 1 ]; then
    say "dry run: nothing will be changed"
    if [ "$has_upstream" -eq 1 ]; then
        read -r behind ahead < <(git rev-list --left-right --count "$REMOTE/$BRANCH...HEAD")
        echo "$BRANCH is $behind behind / $ahead ahead of $REMOTE/$BRANCH"
        if [ "$behind" -gt 0 ]; then
            git log --oneline --no-decorate "HEAD..$REMOTE/$BRANCH" | sed 's/^/     /'
        fi
    else
        echo "$BRANCH has no $REMOTE/$BRANCH; pull step would be skipped"
    fi
    if [ "$MERGE_MAIN" -eq 1 ] && [ "$BRANCH" != "$FROM" ] && [ "$has_from" -eq 1 ]; then
        n=$(git rev-list --count "HEAD..$REMOTE/$FROM")
        echo "merging $REMOTE/$FROM would bring in $n commit(s)"
        if [ "$n" -gt 0 ]; then
            git log --oneline --no-decorate "HEAD..$REMOTE/$FROM" | head -20 | sed 's/^/     /'
        fi
    fi
    exit 0
fi

# ---------------------------------------------------------------- stash
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
    if [ "$STASH" -eq 0 ]; then
        die "working tree has local edits and --no-stash was given. Commit or stash them first."
    fi
    say "stashing local edits"
    git stash push --quiet -m "sync_upstream autostash $(date +%Y-%m-%dT%H:%M:%S)"
    STASHED=1
    echo "saved as: $(git stash list | head -1)"
fi

# ---------------------------------------------------------------- 2. pull
if [ "$has_upstream" -eq 1 ]; then
    read -r behind ahead < <(git rev-list --left-right --count "$REMOTE/$BRANCH...HEAD")
    if [ "$behind" -eq 0 ]; then
        say "$BRANCH already up to date with $REMOTE/$BRANCH (ahead $ahead)"
    elif [ "$REBASE" -eq 1 ]; then
        say "rebasing $BRANCH onto $REMOTE/$BRANCH ($behind new, $ahead local)"
        git rebase "$REMOTE/$BRANCH" || conflict_stop "rebasing onto $REMOTE/$BRANCH"
    elif [ "$ahead" -eq 0 ]; then
        say "fast-forwarding $BRANCH to $REMOTE/$BRANCH ($behind new commit(s))"
        git merge --ff-only --quiet "$REMOTE/$BRANCH"
    else
        say "$BRANCH has diverged ($behind remote, $ahead local): merging $REMOTE/$BRANCH"
        git merge --no-edit --quiet "$REMOTE/$BRANCH" || conflict_stop "merging $REMOTE/$BRANCH"
    fi
else
    say "no $REMOTE/$BRANCH on the remote; skipping the pull step"
fi

# ---------------------------------------------------------------- 3. merge main
MERGED_FROM=0
if [ "$MERGE_MAIN" -eq 1 ] && [ "$BRANCH" != "$FROM" ]; then
    if [ "$has_from" -eq 0 ]; then
        warn "no $REMOTE/$FROM on the remote; skipping the merge step"
    elif git merge-base --is-ancestor "$REMOTE/$FROM" HEAD; then
        say "$BRANCH already contains $REMOTE/$FROM; nothing to merge"
    else
        n=$(git rev-list --count "HEAD..$REMOTE/$FROM")
        say "merging $REMOTE/$FROM into $BRANCH ($n commit(s))"
        git merge --no-edit --quiet "$REMOTE/$FROM" || conflict_stop "merging $REMOTE/$FROM"
        MERGED_FROM=1
    fi
fi

restore_stash

# ---------------------------------------------------------------- 4. report
END=$(git rev-parse HEAD)
say "summary"
if [ "$START" = "$END" ]; then
    echo "nothing new; $BRANCH is unchanged at ${END:0:9}"
else
    new=$(git rev-list --count "$START..$END")
    echo "$BRANCH moved ${START:0:9} -> ${END:0:9} ($new commit(s) in)"
    git diff --shortstat "$START" "$END" | sed 's/^/    /'
    changed=$(git diff --name-only "$START" "$END")
    hint() { printf '  * %s\n' "$*"; }
    if echo "$changed" | grep -qE '^(build\.gradle|settings\.gradle|vendordeps/|gradle/)'; then
        hint "build or vendordeps changed: ./gradlew build (first run downloads new deps)"
    fi
    if echo "$changed" | grep -qE '^src/main/deploy/pathplanner/'; then
        hint "PathPlanner files changed: check auto names in Auton.java match the .auto files exactly"
    fi
    if echo "$changed" | grep -qE '^src/main/java/frc/robot/configs/'; then
        hint "robot configs changed: CAN IDs / offsets may differ, review before deploying"
    fi
    if echo "$changed" | grep -qE '^src/main/java/frc/robot/(State|RobotStates|Coordinator)\.java'; then
        hint "state machine files changed: re-read State/RobotStates/Coordinator before editing them"
    fi
    if echo "$changed" | grep -qE '^docs/|^AGENTS\.md'; then
        hint "docs or AGENTS.md changed: re-read the relevant page before acting on old assumptions"
    fi
    if echo "$changed" | grep -qE '^\.agents/skills/'; then
        hint "agent skills changed"
    fi
    echo "run ./gradlew build before trusting the merged tree."
fi
if [ "$has_upstream" -eq 1 ]; then
    read -r behind ahead < <(git rev-list --left-right --count "$REMOTE/$BRANCH...HEAD")
    echo "vs $REMOTE/$BRANCH: $ahead ahead, $behind behind. Nothing was pushed."
    if [ "$MERGED_FROM" -eq 1 ]; then
        echo "the merge of $REMOTE/$FROM is a local commit; push only when asked."
    fi
fi
if [ "$STASHED" -eq 2 ]; then
    exit 1
fi
exit 0
