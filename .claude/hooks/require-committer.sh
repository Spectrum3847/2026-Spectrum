#!/usr/bin/env bash
# PreToolUse(Bash) hook: refuse any commit that does not name a human author.
#
# Exit 2 denies the tool call and shows stderr to the agent as the reason.
# Deliberately dependency-free (no jq): it substring-matches the raw hook payload,
# so it works anywhere bash does, including Git Bash on the team Windows laptops.

set -uo pipefail

payload="$(cat)"

# Newlines inside the command arrive JSON-escaped as the two characters \n, so a multi-line
# script is one long line here. Turn them back into real newlines (pure bash, no jq) so the
# line-bounded match below can tell "one command" from "several commands in a script".
payload="${payload//\\n/$'\n'}"

# Is this a commit attempt? Match a git invocation whose subcommand is commit, allowing any
# number of -c options in between ("git -c user.name=... commit"). The character class stops
# the match at a command separator or a newline, so a multi-command script that happens to
# mention both words in different places is left alone.
# "commit" must end there: a trailing word character means this is "committer", "commit-graph",
# or a flag like --no-commit-id, none of which write a commit.
is_commit=$'git[^&|;\n]*commit([^a-zA-Z0-9_-]|$)'
if [[ ! "$payload" =~ $is_commit ]]; then
    exit 0
fi

# Identity supplied inline for this one command: allowed.
if [[ "$payload" == *"-c user.name="* && "$payload" == *"-c user.email="* ]]; then
    # ...unless it is an agent identity wearing a hat.
    if [[ "$payload" == *"noreply@anthropic.com"* || "$payload" == *"user.name=Claude"* ]]; then
        echo "Blocked: commits must be authored by a student, not by an agent identity." >&2
        echo "Pick the right person from .claude/committers.json and use their GitHub no-reply email." >&2
        exit 2
    fi
    exit 0
fi

cat >&2 <<'MSG'
Blocked: this commit does not name its author.

Team computers are shared, so the machine's git identity is not trustworthy. Every commit
must carry the author inline.

Do this:
1. Read .claude/committers.json for the roster.
2. Ask the person at the keyboard which entry is theirs. Do not guess, and do not reuse a
   name from earlier in the repo history. If they are not on the roster, ask for their name
   and GitHub no-reply email (GitHub -> Settings -> Emails), and offer to add them.
3. Re-run the commit with the identity attached:

   git -c user.name="Their Name" -c user.email="id+user@users.noreply.github.com" commit -m "..."

Do not work around this by running `git config user.name` or `git config user.email`; that
changes the shared machine for the next student.
MSG
exit 2
