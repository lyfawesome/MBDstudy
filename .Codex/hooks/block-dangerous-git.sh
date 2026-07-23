#!/bin/bash
# Block dangerous git commands before they execute.
# Source: mattpocock/skills (github.com/mattpocock/skills)
#
# Usage: register as a PreToolUse hook in Claude Code settings.json:
# {
#   "hooks": {
#     "PreToolUse": [
#       {
#         "matcher": "Bash",
#         "command": "~/.claude/hooks/block-dangerous-git.sh"
#       }
#     ]
#   }
# }

INPUT=$(cat)
COMMAND=$(echo "$INPUT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('tool_input',{}).get('command',''))" 2>/dev/null || echo "")

if [ -z "$COMMAND" ]; then
  exit 0
fi

DANGEROUS_PATTERNS=(
  "git push"
  "git reset --hard"
  "git clean -fd"
  "git clean -f"
  "git branch -D"
  "git checkout \."
  "git restore \."
  "push --force"
  "reset --hard"
)

for pattern in "${DANGEROUS_PATTERNS[@]}"; do
  if echo "$COMMAND" | grep -qE "$pattern"; then
    cat >&2 <<EOF
========================================
  BLOCKED: Dangerous Git Command
========================================
  Command: $COMMAND
  Matched: $pattern

  This command has been blocked by the
  git safety guardrail hook.

  If you need to run this, ask the user
  for explicit confirmation first.
========================================
EOF
    exit 2
  fi
done

exit 0
