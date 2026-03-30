#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPOS_FILE="${REPO_ROOT}/drs.repos"

missing_deps=()

if ! command -v git &>/dev/null; then
    missing_deps+=("git")
fi

if ! command -v vcs &>/dev/null; then
    missing_deps+=("vcs (vcstool)")
fi

if [ ${#missing_deps[@]} -gt 0 ]; then
    echo "Error: The following required commands are not installed:" >&2
    for dep in "${missing_deps[@]}"; do
        echo "  - ${dep}" >&2
    done
    echo "" >&2
    echo "Install them with:" >&2
    echo "  sudo apt install git python3-vcstool" >&2
    echo "  # or: pip install vcstool" >&2
    exit 1
fi

GITHUB_TOKEN=""
RECURSIVE=false

while [ $# -gt 0 ]; do
    case "$1" in
    --token | -t)
        GITHUB_TOKEN="${2:?--token requires a value}"
        shift 2
        ;;
    --recursive | -r)
        RECURSIVE=true
        shift
        ;;
    *)
        echo "Usage: $0 --token|-t <GITHUB_TOKEN> [--recursive|-r]" >&2
        exit 1
        ;;
    esac
done

if [ -z "${GITHUB_TOKEN}" ]; then
    echo "Error: --token|-t <GITHUB_TOKEN> is required." >&2
    echo "Usage: $0 --token|-t <GITHUB_TOKEN>" >&2
    exit 1
fi

VCS_ARGS=(--force)
if [ "${RECURSIVE}" = true ]; then
    VCS_ARGS+=(--recursive)
fi

GIT_CONFIG_COUNT=1 \
    GIT_CONFIG_KEY_0="url.https://x-access-token:${GITHUB_TOKEN}@github.com/.insteadOf" \
    GIT_CONFIG_VALUE_0="https://github.com/" \
    vcs import "${VCS_ARGS[@]}" "${REPO_ROOT}/src" <"${REPOS_FILE}"
