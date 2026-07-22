#!/usr/bin/env bash
# Run the full verification pipeline locally, the same checks CI runs.
#
# Usage:
#   scripts/verify.sh          check only (fails if formatting is off)
#   scripts/verify.sh --fix    auto-apply formatting first, then verify
#
# spotlessCheck -> formatting;  build -> compile + SpotBugs static analysis + JUnit tests.
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ "${1:-}" == "--fix" ]]; then
    ./gradlew spotlessApply
fi

./gradlew spotlessCheck build
