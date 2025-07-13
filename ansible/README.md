# Data Recording System - Ansible Playbooks

This directory contains Ansible playbooks and roles for deploying the Data Recording System (DRS).

## Quick Start

### Local Installation

This project is designed to be copied to each host and run locally.

```bash
# Run the installation (auto-detects ECU from hostname)
./run_local.sh
```

The script will:
1. Install Ansible if not already installed (via pip3)
2. Detect ECU ID from hostname (looks for 'ecu0' or 'ecu1' in hostname)
3. Apply the appropriate ECU configuration
4. Run the Ansible playbook locally

### Install with specific tags

To install only specific components:

```bash
# Install only storage components
./run_local.sh --tags storage

# Install only DRS components
./run_local.sh --tags drs

# Install only time synchronization
./run_local.sh --tags time
```

### Manual execution

If you need to run ansible-playbook manually:

```bash
# For ECU0
ansible-playbook -i inventory/localhost.yaml -e @inventory/host_vars/ecu0.yaml site.yaml

# For ECU1
ansible-playbook -i inventory/localhost.yaml -e @inventory/host_vars/ecu1.yaml site.yaml
```

### Available Tags

- `storage`: SSD and NAS mount configuration
- `system`: System-level configurations (Jetpack, journald)
- `network`: Network configuration (netplan)
- `time`: Time synchronization (NTP, PTP)
- `ros2`: ROS2 installation
- `docker`: Docker installation and configuration
- `drs`: DRS core components
- `environment`: DRS environment setup
- `middleware`: CycloneDDS configuration
- `build`: DRS build and installation
- `services`: DRS systemd services
- `recorder`: DRS recorder service
- `sensor`: DRS sensor service
- `drivers`: Hardware drivers (camera)
- `camera`: Camera-related drivers and tools
- `c2`: C2 readout delay setter utilities
- `trigger`: Sensor trigger configuration
- `nas`: NAS mounting
- `sync`: Data synchronization to NAS

### Skip specific roles

```bash
# Skip network configuration
./run_local.sh --skip-tags network

# Skip time synchronization
./run_local.sh --skip-tags time
```

### Dry run

To see what changes would be made without applying them:

```bash
./run_local.sh --check
```

## Configuration

### Global Variables

Edit `inventory/group_vars/all.yaml` to configure:
- DRS IDs (ECU, sensing system, module)
- ROS2 domain ID
- CycloneDDS parameters
- NAS mount settings
- Time synchronization settings

### ECU-specific Variables

ECU-specific configurations are stored in:
- `inventory/host_vars/ecu0.yaml` - ECU0 configuration
- `inventory/host_vars/ecu1.yaml` - ECU1 configuration

These files contain:
- ECU ID
- Network interface configurations
- Netplan file selections

## Role Dependencies

The playbook executes roles in this order based on dependencies:

1. **Storage Setup**: `ssd_mount`
2. **System Configuration**: `jetpack`, `journald`, `netplan`, `ntp`, `ptp`
3. **Core Software**: `ros2`, `docker`
4. **DRS Environment**: `drs_env`
5. **Middleware**: `cyclonedds`
6. **DRS Build**: `drs`
7. **Services**: `drs_recorder_service`, `drs_sensor_service`
8. **Additional Components**: `tier4_hdr_camera_driver`, `sensor_trigger`
9. **Data Management**: `nas_mount`, `sync_to_nas`

## Environment Variables

The playbook supports environment variables for configuration:

```bash
# Override ECU ID (if hostname detection fails)
DRS_ECU_ID=0 ./run_local.sh

# Override sensing system ID
SENSING_SYSTEM_ID=aabbccdd ./run_local.sh

# Override module ID
MODULE_ID=eeffgghh ./run_local.sh
```

## Prerequisites

The script will automatically install required dependencies:
- python3-pip (if not installed)
- ansible (if not installed)

You only need:
- Python 3 (usually pre-installed)
- sudo access for system configuration

## Troubleshooting

### Check service status

After installation, verify services are running:

```bash
ssh user@host systemctl status drs_recorder.service
ssh user@host systemctl status drs_sensor.service
```

### View logs

```bash
ssh user@host journalctl -u drs_recorder.service -f
ssh user@host journalctl -u drs_sensor.service -f
```

### Rerun specific roles

If a role fails, you can rerun just that role:

```bash
ansible-playbook -i inventory/hosts.yaml site.yaml --tags <role_tag>
```