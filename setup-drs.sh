#!/bin/bash

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")

# For Python packages installed with user privileges
export PATH="$HOME/.local/bin:$PATH"

echo -e "\e[36mRunning ansible playbook to setup ECU.\e[0m"

# Run ansible
if ansible-playbook "${SCRIPT_DIR}"/ansible/setup-drs.yaml --ask-become-pass; then
    echo -e "\e[32mCompleted.\e[0m"
    exit 0
else
    echo -e "\e[31mFailed.\e[0m"
    exit 1
fi
