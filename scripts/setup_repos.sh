#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPOS_FILE="${REPO_ROOT}/drs.repos"

GITHUB_TOKEN=""

while [ $# -gt 0 ]; do
    case "$1" in
    --token | -t)
        GITHUB_TOKEN="${2:?--token requires a value}"
        shift 2
        ;;
    *)
        echo "Usage: $0 --token|-t <GITHUB_TOKEN>" >&2
        exit 1
        ;;
    esac
done

if [ -z "${GITHUB_TOKEN}" ]; then
    echo "Error: --token|-t <GITHUB_TOKEN> is required." >&2
    echo "Usage: $0 --token|-t <GITHUB_TOKEN>" >&2
    exit 1
fi

GIT_CONFIG_COUNT=1 \
    GIT_CONFIG_KEY_0="url.https://x-access-token:${GITHUB_TOKEN}@github.com/.insteadOf" \
    GIT_CONFIG_VALUE_0="https://github.com/" \
    vcs import --recursive --force "${REPO_ROOT}" <"${REPOS_FILE}"
