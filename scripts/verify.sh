#!/usr/bin/env bash
set -e

# Change to repo root
cd "$(dirname "$0")/.."

if [ "$1" = "--fix" ]; then
    echo "=== Running Spotless Apply ==="
    ./gradlew spotlessApply
else
    echo "=== Running Spotless Check ==="
    ./gradlew spotlessCheck
fi

echo "=== Running SpotBugs Static Analysis ==="
./gradlew spotbugsMain

echo "=== Running Compile and Tests (Unit & Sim Tests) ==="
./gradlew test -Pheadless=true

echo "=== Verification SUCCESSFUL ==="
