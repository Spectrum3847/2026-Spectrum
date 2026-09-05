#!/usr/bin/env bash
#
# Move robot logs off the roboRIO and into the 2026-Robot-Logs repo.
#
# The rio had 2.2 GB of logs on it on 2026-09-05, 53 files, one of them 103 MB.
# Nothing prunes them, and every megabyte written also costs a megabyte of page
# cache -- which is what made "Memory Free" on the Driver Station look like a
# leak all day.
#
# Logs go to GitHub *releases* rather than into git history. They compress about
# 2x, so the backlog is still ~1.1 GB; committed, that would live in the repo
# forever and every clone would pay for it. Release assets do not enter history,
# and `gh release download` gets them back.
#
# Nothing is deleted from the rio unless you pass --delete, and then only after
# the upload has been verified.
#
# Usage:
#   tools/archive-logs.sh                 # copy and upload, leave the rio alone
#   tools/archive-logs.sh --delete        # ...then remove what was uploaded
#   tools/archive-logs.sh --dry-run       # list what it would do, touch nothing
#   tools/archive-logs.sh --keep-local    # leave the working copy behind
#
set -euo pipefail

HOST="${RIO_HOST:-10.85.15.2}"
USER_NAME="${RIO_USER:-lvuser}"
REMOTE_DIR="${RIO_LOG_DIR:-/home/lvuser/logs}"
REPO="${LOG_REPO:-Spectrum3847/2026-Robot-Logs}"
# Files this new are skipped: robot code may still be writing them.
MIN_AGE_MINUTES="${MIN_AGE_MINUTES:-5}"

DELETE=0
DRY_RUN=0
KEEP_LOCAL=0
while [ $# -gt 0 ]; do
    case "$1" in
        --delete) DELETE=1 ;;
        --dry-run) DRY_RUN=1 ;;
        --keep-local) KEEP_LOCAL=1 ;;
        --host) HOST="$2"; shift ;;
        --repo) REPO="$2"; shift ;;
        -h|--help) sed -n '2,26p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

say() { printf '%s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

for tool in ssh scp gzip gh; do
    command -v "$tool" >/dev/null 2>&1 || die "$tool is not on PATH"
done
gh auth status >/dev/null 2>&1 || die "gh is not logged in; run: gh auth login"

SSH_OPTS=(-o ConnectTimeout=15 -o StrictHostKeyChecking=no -o BatchMode=yes)
rio() { ssh "${SSH_OPTS[@]}" "$USER_NAME@$HOST" "$@"; }

say "roboRIO  $USER_NAME@$HOST:$REMOTE_DIR"
rio true 2>/dev/null || die "cannot reach the roboRIO at $HOST (is it powered and on the network?)"

# Both kinds of log the robot writes: WPILib data logs and Phoenix signal logs.
# Sizes come back with the listing: the rio runs busybox, whose du has no -b, and
# asking per file would be one SSH round trip each across ninety of them.
say "Looking for logs at least $MIN_AGE_MINUTES minutes old..."
mapfile -t LISTING < <(
    rio "find '$REMOTE_DIR' -type f \\( -name '*.wpilog' -o -name '*.hoot' \\) -mmin +$MIN_AGE_MINUTES -exec stat -c '%s %n' {} +" \
        2>/dev/null | tr -d '\r' | sed '/^$/d'
)

REMOTE_FILES=()
declare -A REMOTE_SIZE=()
REMOTE_BYTES=0
for line in "${LISTING[@]}"; do
    size="${line%% *}"
    path="${line#* }"
    [ -n "$path" ] && [ "$size" -eq "$size" ] 2>/dev/null || continue
    REMOTE_FILES+=("$path")
    REMOTE_SIZE["$path"]="$size"
    REMOTE_BYTES=$(( REMOTE_BYTES + size ))
done

if [ "${#REMOTE_FILES[@]}" -eq 0 ]; then
    say "Nothing to archive."
    exit 0
fi

say "Found ${#REMOTE_FILES[@]} file(s), $(( REMOTE_BYTES / 1048576 )) MiB."

TAG="logs-$(date +%Y-%m-%d)"
if [ "$DRY_RUN" -eq 1 ]; then
    say ""
    say "DRY RUN. Would upload to $REPO release '$TAG':"
    for f in "${REMOTE_FILES[@]}"; do say "    $(basename "$f")"; done
    [ "$DELETE" -eq 1 ] && say "Would then delete all ${#REMOTE_FILES[@]} from the rio."
    exit 0
fi

WORK="$(mktemp -d)"
cleanup() {
    if [ "$KEEP_LOCAL" -eq 1 ]; then
        say "Working copy left at $WORK"
    else
        rm -rf "$WORK"
    fi
}
trap cleanup EXIT

say ""
say "Downloading to $WORK ..."
DOWNLOADED=()
for remote in "${REMOTE_FILES[@]}"; do
    base="$(basename "$remote")"
    local_path="$WORK/$base"
    if ! scp "${SSH_OPTS[@]}" -q "$USER_NAME@$HOST:$remote" "$local_path" 2>/dev/null; then
        say "  SKIP $base (copy failed)"
        continue
    fi
    # Verify byte-for-byte size before this file is a candidate for deletion.
    want="${REMOTE_SIZE[$remote]}"
    got=$(stat -c%s "$local_path")
    if [ "$want" != "$got" ]; then
        say "  SKIP $base (size mismatch: rio $want, local $got)"
        rm -f "$local_path"
        continue
    fi
    gzip -6 "$local_path"
    say "  ok   $base  ($(( got / 1048576 )) MiB -> $(( $(stat -c%s "$local_path.gz") / 1048576 )) MiB)"
    DOWNLOADED+=("$remote")
done

if [ "${#DOWNLOADED[@]}" -eq 0 ]; then
    die "nothing copied successfully; the rio was left untouched"
fi

say ""
say "Uploading ${#DOWNLOADED[@]} file(s) to $REPO release '$TAG' ..."
if ! gh release view "$TAG" --repo "$REPO" >/dev/null 2>&1; then
    gh release create "$TAG" --repo "$REPO" \
        --title "Robot logs $(date +%Y-%m-%d)" \
        --notes "WPILib data logs and Phoenix signal logs pulled off the roboRIO. Gzipped; \`gunzip\` to read." \
        >/dev/null
    say "  created release $TAG"
fi
gh release upload "$TAG" --repo "$REPO" --clobber "$WORK"/*.gz

# Confirm every asset is actually on the release before anything is removed.
say ""
say "Verifying ..."
mapfile -t ASSETS < <(gh release view "$TAG" --repo "$REPO" --json assets --jq '.assets[].name')
MISSING=0
for remote in "${DOWNLOADED[@]}"; do
    want="$(basename "$remote").gz"
    printf '%s\n' "${ASSETS[@]}" | grep -qxF "$want" || { say "  MISSING on release: $want"; MISSING=1; }
done
[ "$MISSING" -eq 0 ] || die "some assets are not on the release; the rio was left untouched"
say "  all ${#DOWNLOADED[@]} asset(s) present on $TAG"

if [ "$DELETE" -eq 1 ]; then
    say ""
    say "Deleting ${#DOWNLOADED[@]} verified file(s) from the rio ..."
    for remote in "${DOWNLOADED[@]}"; do
        rio "rm -f '$remote'"
    done
    # Phoenix signal logs live in dated directories; drop the empty ones behind them.
    rio "find '$REMOTE_DIR' -mindepth 1 -type d -empty -delete" 2>/dev/null || true
    say "Remaining on the rio:"
    rio "du -sh '$REMOTE_DIR'; ls '$REMOTE_DIR' | wc -l" | tr -d '\r' | sed 's/^/  /'
else
    say ""
    say "Left on the rio. Re-run with --delete to remove the ${#DOWNLOADED[@]} file(s) now archived."
fi

say ""
say "Done: https://github.com/$REPO/releases/tag/$TAG"
