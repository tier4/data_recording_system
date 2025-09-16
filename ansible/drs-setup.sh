#!/bin/bash
# Run Ansible playbook on local host with automatic ECU detection

set -e

echo "Data Recording System - Local Installation Script"
echo "================================================"

# Check sudo access early
echo "Checking sudo access..."
if ! sudo -n true 2>/dev/null; then
    echo "This script requires sudo access for system configuration."
    echo "Please enter your password when prompted."
    sudo true || {
        echo "ERROR: Failed to obtain sudo access."
        exit 1
    }
fi
echo "Sudo access confirmed."

# Check if pip3 is installed, if not, install it
if ! command -v pip3 &> /dev/null; then
    echo "pip3 not found. Installing python3-pip..."
    sudo apt-get update
    sudo apt-get install -y python3-pip
    
    if ! command -v pip3 &> /dev/null; then
        echo "ERROR: Failed to install pip3. Please install python3-pip manually."
        exit 1
    fi
    echo "pip3 installed successfully."
fi

# Check if ansible is installed, if not, install it
if ! command -v ansible-playbook &> /dev/null; then
    echo "Ansible not found. Installing ansible..."
    pip3 install --user ansible
    
    # Add user's pip bin directory to PATH if not already there
    if [[ ":$PATH:" != *":$HOME/.local/bin:"* ]]; then
        export PATH="$HOME/.local/bin:$PATH"
    fi
    
    # Verify installation
    if ! command -v ansible-playbook &> /dev/null; then
        echo "ERROR: Failed to install ansible. Please install it manually."
        exit 1
    fi
    echo "Ansible installed successfully."
else
    echo "Ansible is already installed."
fi

# Detect device type and select playbook
HOSTNAME=$(hostname)
PLAYBOOK=""

# Check for Raspberry Pi
if [[ -e /proc/device-tree/model ]] && grep -q "Raspberry" /proc/device-tree/model 2>/dev/null; then
    PLAYBOOK="drs-control.yaml"
    echo "Detected Raspberry Pi - using control module playbook"
else
    # Default to sensing module (Jetson/x86)
    PLAYBOOK="drs-sensing.yaml"
    echo "Using sensing module playbook (Jetson/x86)"
fi

# Override playbook with environment variable if set
if [[ -n "$DRS_PLAYBOOK" ]]; then
    PLAYBOOK="$DRS_PLAYBOOK"
    echo "Overriding with playbook from environment: $PLAYBOOK"
fi

# Detect ECU ID for inventory
if [[ $HOSTNAME == *"ecu0"* ]]; then
    echo "Detected ECU0 from hostname"
    ECU_ID=0
    ECU_VARS="inventory/host_vars/ecu0.yaml"
elif [[ $HOSTNAME == *"ecu1"* ]]; then
    echo "Detected ECU1 from hostname"
    ECU_ID=1
    ECU_VARS="inventory/host_vars/ecu1.yaml"
elif [[ -n "$DRS_ECU_ID" ]]; then
    echo "Using ECU ID from environment variable: $DRS_ECU_ID"
    ECU_ID=$DRS_ECU_ID
    ECU_VARS="inventory/host_vars/ecu${ECU_ID}.yaml"
else
    # For control modules, ECU ID might not be needed
    if [[ "$PLAYBOOK" == "drs-control.yaml" ]]; then
        echo "Control module detected - ECU ID not required"
        ECU_ID=""
        ECU_VARS=""
    else
        echo "ERROR: Could not detect ECU ID from hostname."
        echo ""
        echo "Your hostname is: $HOSTNAME"
        echo ""
        echo "Please either:"
        echo "  1. Set DRS_ECU_ID environment variable:"
        echo "     DRS_ECU_ID=0 $0"
        echo "  2. Ensure hostname contains 'ecu0' or 'ecu1'"
        exit 1
    fi
fi

# Check if ECU vars file exists
if [[ ! -f "$ECU_VARS" ]]; then
    echo "WARNING: ECU variables file not found: $ECU_VARS"
    echo "Using default values."
    ECU_VARS=""
else
    echo "Using ECU configuration: $ECU_VARS"
fi

# Set environment variables
export DRS_ECU_ID=$ECU_ID
export ANSIBLE_HOST_KEY_CHECKING=False

echo ""
if [[ -n "$ECU_ID" ]]; then
    echo "Starting Ansible playbook for ECU${ECU_ID}..."
else
    echo "Starting Ansible playbook..."
fi
echo "Playbook: $PLAYBOOK"
echo ""

# Run the playbook
if [[ -n "$ECU_VARS" ]]; then
    ansible-playbook \
        -i inventory/localhost.yaml \
        -e @"$ECU_VARS" \
        "$PLAYBOOK" \
        "$@"
else
    ansible-playbook \
        -i inventory/localhost.yaml \
        "$PLAYBOOK" \
        "$@"
fi