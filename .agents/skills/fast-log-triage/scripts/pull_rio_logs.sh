#!/usr/bin/env bash
# Copy the newest .wpilog files off the Spectrum 3847 roboRIO into a local folder.
#
#   pull_rio_logs.sh [-n COUNT] [-H HOST] [DEST]
#
# Tries the robot radio address (10.38.47.2), then USB (172.22.11.2), then mDNS.
# Logs live in /U/logs (USB stick) or /home/lvuser/logs on the RIO. Read-only: nothing on
# the RIO is modified. lvuser has no password; sshpass is used when present so the copy
# does not stop to prompt.
set -euo pipefail

COUNT=1
HOSTS=(10.38.47.2 172.22.11.2 roboRIO-3847-FRC.local)
DEST="rio-logs"
while getopts "n:H:h" opt; do
    case "$opt" in
        n) COUNT="$OPTARG" ;;
        H) HOSTS=("$OPTARG") ;;
        h) sed -n '2,10p' "$0"; exit 0 ;;
        *) exit 2 ;;
    esac
done
shift $((OPTIND - 1))
[ $# -ge 1 ] && DEST="$1"
mkdir -p "$DEST"

SSH_OPTS=(-o BatchMode=no -o ConnectTimeout=3 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR)
if command -v sshpass >/dev/null 2>&1; then
    RUN=(sshpass -p "")
else
    RUN=()
fi

for host in "${HOSTS[@]}"; do
    echo "trying lvuser@$host ..." >&2
    if ! files=$("${RUN[@]}" ssh "${SSH_OPTS[@]}" "lvuser@$host" \
        "ls -t /U/logs/*.wpilog /home/lvuser/logs/*.wpilog 2>/dev/null | head -n $COUNT"); then
        continue
    fi
    if [ -z "$files" ]; then
        echo "connected to $host but found no .wpilog files" >&2
        exit 1
    fi
    for f in $files; do
        echo "copying $f" >&2
        "${RUN[@]}" scp "${SSH_OPTS[@]}" "lvuser@$host:$f" "$DEST/"
    done
    echo "$DEST"
    ls -t "$DEST"/*.wpilog | head -n "$COUNT"
    exit 0
done
echo "could not reach the roboRIO on: ${HOSTS[*]}" >&2
exit 1
