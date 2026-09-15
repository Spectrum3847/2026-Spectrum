#!/usr/bin/env bash
# Test matrix for .claude/hooks/require-committer.sh
S="$(dirname "${BASH_SOURCE[0]}")/require-committer.sh"
pass=0; fail=0

check() { # $1=label $2=want $3=command-string
  local payload got
  payload=$(printf '{"tool_name":"Bash","tool_input":{"command":%s}}' "$(printf '%s' "$3" | python3 -c 'import json,sys;print(json.dumps(sys.stdin.read()))')")
  printf '%s' "$payload" | bash "$S" >/dev/null 2>&1
  got=$?
  if [ "$got" = "$2" ]; then pass=$((pass+1)); printf 'PASS  %-34s exit=%s\n' "$1" "$got"
  else fail=$((fail+1)); printf 'FAIL  %-34s want=%s got=%s\n' "$1" "$2" "$got"; fi
}

G=git; C=commit
check "student identity"        0 "$G -c user.name=\"Jane Doe\" -c user.email=\"1+jane@users.noreply.github.com\" $C -m \"Add turret telemetry\""
check "agent identity"          2 "$G -c user.name=Claude -c user.email=noreply@anthropic.com $C -m x"
check "bare commit"             2 "$G $C -m wip"
check "compound amend"          2 "cd src && $G $C --amend --no-edit"
check "unrelated build"         0 "./gradlew build"
check "git log"                 0 "$G log --oneline -5"
check "separator false positive" 0 "$G status && echo $C"
check "multiline false positive" 0 "$(printf '%s log -1\necho done\necho %s here' "$G" "$C")"
check "identity with add"       0 "$G add -A && $G -c user.name=\"A B\" -c user.email=\"2+ab@users.noreply.github.com\" $C -m msg"
# "commit" as a prefix of a longer word is not a commit: these are reads, not writes.
check "log format committer"    0 "$G log -1 --format='%an <%ae> / ${C}ter %cn'"
check "commit-graph"            0 "$G ${C}-graph verify"
check "no-commit-id flag"       0 "$G diff-tree --no-${C}-id -r HEAD"
check "commit at end of string"  2 "$G $C"

printf '\n%d passed, %d failed\n' "$pass" "$fail"
[ "$fail" = 0 ]
