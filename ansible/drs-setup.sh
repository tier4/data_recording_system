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
if ! command -v pip3 &>/dev/null; then
    echo "pip3 not found. Installing python3-pip..."
    sudo apt-get update
    sudo apt-get install -y python3-pip

    if ! command -v pip3 &>/dev/null; then
        echo "ERROR: Failed to install pip3. Please install python3-pip manually."
        exit 1
    fi
    echo "pip3 installed successfully."
fi

# Check if pipx is installed, if not, install it
if ! command -v pipx &>/dev/null; then
    echo "pipx not found. Installing pipx..."
    sudo apt-get update
    sudo apt-get install -y pipx

    if ! command -v pipx &>/dev/null; then
        echo "ERROR: Failed to install pipx. Please install pipx manually."
        exit 1
    fi
    echo "pipx installed successfully."

    # Ensure pipx bin directory is in PATH
    pipx ensurepath
    export PATH="$HOME/.local/bin:$PATH"
fi

# Check if ansible is installed, if not, install it with pipx
if ! command -v ansible-playbook &>/dev/null; then
    echo "Ansible not found. Installing ansible with pipx..."
    # Check if pipx thinks ansible-core is already installed
    if pipx list --short | grep -q "^ansible-core "; then
        echo "Ansible seems to be installed via pipx but command is missing."
        echo "Attempting to reinstall/repair..."
        pipx reinstall ansible-core
    else
        # Clean install
        pipx install --include-deps ansible-core
    fi

    # Verify installation
    if ! command -v ansible-playbook &>/dev/null; then
        echo "ERROR: Failed to install ansible. Please install it manually."
        exit 1
    fi
    echo "Ansible installed successfully with pipx."
else
    echo "Ansible is already installed."
fi

# Install Ansible Galaxy requirements
echo "Installing Ansible Galaxy requirements..."
if [[ -f requirements.yml ]]; then
    if ansible-galaxy collection install -r requirements.yml --upgrade; then
        echo "Ansible Galaxy requirements installed successfully."
    else
        echo "WARNING: Failed to install some Ansible Galaxy requirements."
        echo "Some playbook features may not work correctly."
    fi
else
    echo "WARNING: requirements.yml not found. Skipping Galaxy requirements installation."
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
if [[ -n $DRS_PLAYBOOK ]]; then
    PLAYBOOK="$DRS_PLAYBOOK"
    echo "Overriding with playbook from environment: $PLAYBOOK"
fi

# Detect target host based on exact hostname match
TARGET_HOST=""
case "$HOSTNAME" in
ecu0)
    echo "Detected ECU0 from hostname"
    TARGET_HOST="ecu0"
    ECU_ID=0
    ;;
ecu1)
    echo "Detected ECU1 from hostname"
    TARGET_HOST="ecu1"
    ECU_ID=1
    ;;
raspi)
    echo "Detected Raspberry Pi from hostname"
    TARGET_HOST="raspi"
    ECU_ID=""
    ;;
*)
    # Allow override with environment variable for testing
    if [[ -n $DRS_TARGET_HOST ]]; then
        echo "Using target host from environment: $DRS_TARGET_HOST"
        TARGET_HOST="$DRS_TARGET_HOST"
        # Set ECU_ID for ecu hosts
        case "$TARGET_HOST" in
        ecu0) ECU_ID=0 ;;
        ecu1) ECU_ID=1 ;;
        raspi) ECU_ID="" ;;
        *) ECU_ID="" ;;
        esac
    else
        echo "ERROR: Hostname must be exactly 'ecu0', 'ecu1', or 'raspi'"
        echo "Current hostname: $HOSTNAME"
        echo ""
        echo "To fix:"
        echo "  1. Set hostname: sudo hostnamectl set-hostname [ecu0|ecu1|raspi]"
        echo "  2. Or override: DRS_TARGET_HOST=[ecu0|ecu1|raspi] $0"
        exit 1
    fi
    ;;
esac

# Set vars file path based on target host
if [[ -n $TARGET_HOST ]]; then
    ECU_VARS="inventory/host_vars/${TARGET_HOST}.yaml"
fi

# Check if ECU vars file exists
if [[ ! -f $ECU_VARS ]]; then
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
if [[ -n $ECU_ID ]]; then
    echo "Starting Ansible playbook for ECU${ECU_ID}..."
else
    echo "Starting Ansible playbook..."
fi
echo "Playbook: $PLAYBOOK"
echo ""

# Run the playbook with target host
if [[ -n $TARGET_HOST ]]; then
    # Use detected/specified host with automatic host_vars loading
    ansible-playbook \
        -i inventory/localhost.yaml \
        -K \
        -e "target_host=$TARGET_HOST" \
        "$PLAYBOOK" \
        "$@"
else
    # Fallback to localhost
    ansible-playbook \
        -i inventory/localhost.yaml \
        -K \
        "$PLAYBOOK" \
        "$@"
fi
