#!/usr/bin/env bash
# SessionStart hook: tell the session how commit authorship works on a shared computer.
#
# Team laptops are used by several students a day. Whatever `git config user.name`
# happens to be set to is almost certainly the last person's, so the session must ask
# rather than inherit. Stdout from a SessionStart hook is added to the session context.

set -uo pipefail

name="$(git config user.name 2>/dev/null || true)"
email="$(git config user.email 2>/dev/null || true)"

echo "## Commit authorship (Spectrum 3847 shared-computer policy)"
echo
if [ -n "$name" ] || [ -n "$email" ]; then
    echo "The git identity currently configured on this machine is \"${name:-<unset>}\" <${email:-<unset>}>."
    echo "Do NOT assume it belongs to the person you are working with. It is probably the last student's."
else
    echo "No git identity is configured on this machine. This is intentional."
fi
echo
echo "Before the first commit of this session:"
echo "1. Read .claude/committers.json for the team roster."
echo "2. Ask the person at the keyboard which entry is theirs (offer the roster names as choices)."
echo "   If they are not on the roster, ask for their name and GitHub no-reply email, and offer to add them."
echo "3. Commit with the identity passed inline, so nothing on the shared machine is changed:"
echo "     git -c user.name=\"Their Name\" -c user.email=\"id+user@users.noreply.github.com\" commit -m \"...\""
echo
echo "Never run 'git config user.name' or 'git config user.email' to set a value, globally or locally."
echo "Never commit under an agent, bot, or Anthropic identity. A commit must be authored by the student."
