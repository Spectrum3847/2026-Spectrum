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
# 2x into zips, so the backlog is still ~1.1 GB; committed, that would live in the repo
# forever and every clone would pay for it. Release assets do not enter history,
# and `gh release download` gets them back. Zip, not tar.gz -- this team is on
# Windows, where a zip opens with a double click and a tarball does not.
#
# Nothing is deleted from the rio unless you pass --delete, and then only after
# the upload has been verified.
#
# Driver Station logs are a separate job: they live on the laptop, not the robot,
# so --ds-logs skips the SSH work entirely and uploads them as one zip.
#
# Usage:
#   tools/archive-logs.sh                 # copy and upload, leave the rio alone
#   tools/archive-logs.sh --delete        # ...then remove what was uploaded
#   tools/archive-logs.sh --dry-run       # list what it would do, touch nothing
#   tools/archive-logs.sh --keep-local    # leave the working copy behind
#   tools/archive-logs.sh --ds-logs [DIR] # DS logs only (default "logs/DS Logs")
#
set -euo pipefail

HOST="${RIO_HOST:-10.85.15.2}"
USER_NAME="${RIO_USER:-lvuser}"
REMOTE_DIR="${RIO_LOG_DIR:-/home/lvuser/logs}"
REPO="${LOG_REPO:-Spectrum3847/2026-Robot-Logs}"
# Files this new are skipped: robot code may still be writing them.
MIN_AGE_MINUTES="${MIN_AGE_MINUTES:-5}"

DS_LOG_DIR="${DS_LOG_DIR:-logs/DS Logs}"

DELETE=0
DRY_RUN=0
KEEP_LOCAL=0
DS_ONLY=0
while [ $# -gt 0 ]; do
    case "$1" in
        --delete) DELETE=1 ;;
        --dry-run) DRY_RUN=1 ;;
        --keep-local) KEEP_LOCAL=1 ;;
        --ds-logs) DS_ONLY=1; [ $# -gt 1 ] && [ "${2#-}" = "$2" ] && { DS_LOG_DIR="$2"; shift; } ;;
        --host) HOST="$2"; shift ;;
        --repo) REPO="$2"; shift ;;
        -h|--help) sed -n '2,29p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

say() { printf '%s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

# Zip, because this team is on Windows and .tar.gz needs a tool to open. Git Bash
# ships no zip binary, so fall back through what is actually likely to be there.
#   make_zip <out.zip> <dir-to-enter> <name-inside>
make_zip() {
    local out="$1" parent="$2" item="$3"
    if command -v zip >/dev/null 2>&1; then
        (cd "$parent" && zip -q -r -9 "$out" "$item")
    elif command -v python >/dev/null 2>&1; then
        python -c '
import os, sys, zipfile
out, parent, item = sys.argv[1:4]
root = os.path.join(parent, item)
with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
    if os.path.isdir(root):
        for base, _, files in os.walk(root):
            for f in files:
                full = os.path.join(base, f)
                z.write(full, os.path.relpath(full, parent))
    else:
        z.write(root, item)
' "$out" "$parent" "$item"
    elif command -v powershell >/dev/null 2>&1; then
        powershell -NoProfile -Command \
            "Compress-Archive -Path '$(cygpath -w "$parent/$item")' -DestinationPath '$(cygpath -w "$out")' -Force"
    else
        die "no way to make a zip: install zip, or have python or powershell on PATH"
    fi
}

# Asset name for a remote path: its location under the log directory with "/"
# turned into "__". Naming by basename alone lost data on 2026-09-05 -- Phoenix
# writes .hoot files into per-session directories, three basenames existed in two
# directories each, --clobber silently overwrote one copy with the other, and the
# delete pass then removed both from the rio.
asset_name_for() {
    local rel="${1#"$REMOTE_DIR"/}"
    printf '%s' "${rel//\//__}"
}

for tool in ssh scp gh; do
    command -v "$tool" >/dev/null 2>&1 || die "$tool is not on PATH"
done
gh auth status >/dev/null 2>&1 || die "gh is not logged in; run: gh auth login"

TAG="logs-$(date +%Y-%m-%d)"

# Create the day's release if it is not there yet. Both paths below upload into it.
ensure_release() {
    if ! gh release view "$TAG" --repo "$REPO" >/dev/null 2>&1; then
        gh release create "$TAG" --repo "$REPO" \
            --title "Robot logs $(date +%Y-%m-%d)" \
            --notes "Logs pulled off the roboRIO and the Driver Station laptop. Each asset is a zip." \
            >/dev/null
        say "  created release $TAG"
    fi
}

# --- Driver Station logs -------------------------------------------------------
# These live on the laptop, not the robot, so none of the SSH work below applies.
# They are small -- 8.2 MB across 54 files on 2026-09-05 -- and only useful as a
# set, so they go up as one zip rather than 54 separate assets.
if [ "$DS_ONLY" -eq 1 ]; then
    [ -d "$DS_LOG_DIR" ] || die "no such directory: $DS_LOG_DIR"
    COUNT=$(find "$DS_LOG_DIR" -type f \( -name '*.dsevents' -o -name '*.dslog' \) | wc -l)
    [ "$COUNT" -gt 0 ] || die "no .dsevents or .dslog files in $DS_LOG_DIR"
    say "Driver Station logs  $DS_LOG_DIR"
    say "Found $COUNT file(s), $(du -sh "$DS_LOG_DIR" | cut -f1)."

    ASSET="ds-logs-$(date +%Y-%m-%d).zip"
    if [ "$DRY_RUN" -eq 1 ]; then
        say ""
        say "DRY RUN. Would upload $ASSET to $REPO release '$TAG'."
        [ "$DELETE" -eq 1 ] && say "Would then delete the $COUNT local file(s)."
        exit 0
    fi

    WORK="$(mktemp -d)"
    trap 'rm -rf "$WORK"' EXIT
    make_zip "$WORK/$ASSET" "$(dirname "$DS_LOG_DIR")" "$(basename "$DS_LOG_DIR")"
    say "Packed $(du -h "$WORK/$ASSET" | cut -f1)."

    ensure_release
    gh release upload "$TAG" --repo "$REPO" --clobber "$WORK/$ASSET"

    # Confirm it is really on the release before removing anything local.
    gh release view "$TAG" --repo "$REPO" --json assets --jq '.assets[].name' \
        | grep -qxF "$ASSET" || die "upload did not land; nothing was deleted"
    say "Verified $ASSET on release $TAG."

    if [ "$DELETE" -eq 1 ]; then
        find "$DS_LOG_DIR" -type f \( -name '*.dsevents' -o -name '*.dslog' \) -delete
        say "Deleted $COUNT local file(s) from $DS_LOG_DIR."
    else
        say "Left in place. Re-run with --delete to clear them off this machine."
    fi

    say ""
    say "Done: https://github.com/$REPO/releases/tag/$TAG"
    exit 0
fi

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

if [ "$DRY_RUN" -eq 1 ]; then
    say ""
    say "DRY RUN. Would upload to $REPO release '$TAG':"
    # Show the asset names, not basenames, so a collision is visible in a dry run.
    for f in "${REMOTE_FILES[@]}"; do say "    $(asset_name_for "$f").zip"; done
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
    base="$(asset_name_for "$remote")"
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
    make_zip "$local_path.zip" "$WORK" "$base"
    rm -f "$local_path"
    say "  ok   $base  ($(( got / 1048576 )) MiB -> $(( $(stat -c%s "$local_path.zip") / 1048576 )) MiB)"
    DOWNLOADED+=("$remote")
done

if [ "${#DOWNLOADED[@]}" -eq 0 ]; then
    die "nothing copied successfully; the rio was left untouched"
fi

# Distinct names, not just present names. The old check asked "is an asset called
# this on the release", which a file that had just been clobbered by its twin
# passes -- so both got deleted. Count first, delete later.
declare -A ASSET_SEEN=()
for remote in "${DOWNLOADED[@]}"; do
    n="$(asset_name_for "$remote").zip"
    [ -z "${ASSET_SEEN[$n]:-}" ]         || die "two files map to the same asset name ($n); nothing was uploaded or deleted"
    ASSET_SEEN["$n"]=1
done
ZIP_COUNT=$(find "$WORK" -maxdepth 1 -name '*.zip' | wc -l)
[ "$ZIP_COUNT" -eq "${#DOWNLOADED[@]}" ]     || die "made $ZIP_COUNT zips for ${#DOWNLOADED[@]} files; nothing was uploaded or deleted"

say ""
say "Uploading ${#DOWNLOADED[@]} file(s) to $REPO release '$TAG' ..."
ensure_release
gh release upload "$TAG" --repo "$REPO" --clobber "$WORK"/*.zip

# Confirm every asset is actually on the release before anything is removed.
say ""
say "Verifying ..."
mapfile -t ASSETS < <(gh release view "$TAG" --repo "$REPO" --json assets --jq '.assets[].name')
MISSING=0
for remote in "${DOWNLOADED[@]}"; do
    want="$(asset_name_for "$remote").zip"
    printf '%s\n' "${ASSETS[@]}" | grep -qxF "$want" || { say "  MISSING on release: $want"; MISSING=1; }
done
[ "$MISSING" -eq 0 ] || die "some assets are not on the release; the rio was left untouched"
say "  all ${#DOWNLOADED[@]} asset(s) present on $TAG"

if [ "$DELETE" -eq 1 ]; then
    say ""
    say "Deleting ${#DOWNLOADED[@]} verified file(s) from the rio ..."
    # One connection, not one per file. The 2026-09-05 run spent about five
    # minutes on 123 SSH handshakes to move no data at all.
    {
        printf "rm -f '%s'\n" "${DOWNLOADED[@]}"
        # Phoenix signal logs live in dated directories; drop the empty ones behind them.
        printf "find '%s' -mindepth 1 -type d -empty -delete\n" "$REMOTE_DIR"
    } | rio "sh -s" >/dev/null 2>&1 || die "delete failed; the logs are safe on $TAG, the rio still has them"
    say "Remaining on the rio:"
    rio "du -sh '$REMOTE_DIR'; ls '$REMOTE_DIR' | wc -l" | tr -d '\r' | sed 's/^/  /'
else
    say ""
    say "Left on the rio. Re-run with --delete to remove the ${#DOWNLOADED[@]} file(s) now archived."
fi

say ""
say "Done: https://github.com/$REPO/releases/tag/$TAG"
