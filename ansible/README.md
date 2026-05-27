# Data Recording System - Ansible Playbooks

This directory contains Ansible playbooks and roles for deploying the Data Recording System (DRS).

## Quick Start

### Local Installation

This project is designed to be copied to each host and run locally.

> **Before running:** Internet access is required for downloading packages and collections.
> System time must be set correctly (certificate validation and package signing depend on it).

```bash
# Run the installation (auto-detects device type and hostname)
./drs-setup.sh
```

During execution you will be prompted for some or all of the following, depending on the detected module type and whether placeholder/default values are still set:

- **sudo password** (`-K`) — required for system-level configuration
- **GitHub Personal Access Token (classic)** — required to fetch private repositories (press Enter to skip if not needed); note that the handling of this token is still under consideration
- **`sensing_system_id` / `module_id`** — for sensing modules when default values are detected and must be customized
- **WiFi AP password** — for control modules when configuring the access point
- **custom hostname** *(optional)* — for control modules if you choose to override the default hostname

The script will:

1. Install `pipx` if not already installed (via apt)
2. Install `ansible-core` via pipx if not already installed
3. Install Ansible Galaxy collections from `requirements.yml`
4. Detect device type (Raspberry Pi → control module, Jetson/x86 → sensing module)
5. Match the hostname exactly (`ecu0`, `ecu1`, or `raspi`) to select host variables
6. Run the appropriate playbook locally

### Playbooks

| Playbook           | Target                  | Description                                       |
| ------------------ | ----------------------- | ------------------------------------------------- |
| `drs-sensing.yaml` | ECU0, ECU1 (Jetson/x86) | Sensing module — full DRS stack                   |
| `drs-control.yaml` | Raspberry Pi (`raspi`)  | Control module — WiFi AP, DHCP, NAT, DRS services |

### Install with specific tags

```bash
# Sensing module: install only storage components
./drs-setup.sh --tags ssd_mount

# Sensing module: install only DRS core
./drs-setup.sh --tags drs

# Sensing module: install only time synchronization
./drs-setup.sh --tags ptp

# Control module: install only network stack
./drs-setup.sh --tags network,hostapd,dhcp,iptables
```

### Environment Variables

```bash
# Override target host (when hostname doesn't match ecu0/ecu1/raspi)
DRS_TARGET_HOST=ecu0 ./drs-setup.sh

# Override playbook selection
DRS_PLAYBOOK=drs-sensing.yaml ./drs-setup.sh

# Override sensing system ID
SENSING_SYSTEM_ID=aabbccdd ./drs-setup.sh

# Override module ID
MODULE_ID=eeffgghh ./drs-setup.sh
```

### Dry run

```bash
./drs-setup.sh --check
```

## Available Tags

### Sensing module (`drs-sensing.yaml`)

| Tag                       | Description                            |
| ------------------------- | -------------------------------------- |
| `cleanup`                 | Pre-installation cleanup               |
| `drs`, `config`           | DRS environment and configuration      |
| `ssd_mount`               | Internal SSD mount                     |
| `jetpack`                 | NVIDIA Jetpack configuration           |
| `jetson`                  | Jetson system tuning (jetson-clocks)   |
| `journald`                | journald log size configuration        |
| `netplan`                 | Network interface configuration        |
| `can`                     | CAN interface setup (ECU0 only)        |
| `iptables`                | Firewall rules                         |
| `ptp`                     | PTP time synchronization               |
| `ntp`, `ntp_server`       | NTP server (opt-in per host)           |
| `ros2`                    | ROS 2 installation                     |
| `docker`                  | Docker (Anvil/Jetson) installation     |
| `cyclonedds`              | CycloneDDS middleware configuration    |
| `drs_recorder_service`    | DRS recorder systemd service           |
| `drs_sensor_service`      | DRS sensor systemd service             |
| `drs_api_service`         | DRS API systemd service                |
| `drs_ros2_bridge_service` | ROS 2 bridge service (opt-in)          |
| `drs_dashboard_service`   | Dashboard service (opt-in)             |
| `tier4_hdr_camera_driver` | TIER IV HDR camera driver              |
| `c2_readout_delay_setter` | C2 readout delay utility               |
| `sensor_trigger`          | Sensor trigger configuration           |
| `external_storage`        | External SSD, NAS mount, data transfer |
| `extra_apps`              | Additional utilities                   |

### Control module (`drs-control.yaml`)

| Tag                        | Description                     |
| -------------------------- | ------------------------------- |
| `raspi`, `raspi_time_sync` | Time synchronization via chrony |
| `raspi`, `network`         | Network interfaces (VLAN)       |
| `raspi`, `hostapd`         | WiFi Access Point               |
| `raspi`, `dhcp`            | DHCP server                     |
| `raspi`, `iptables`        | Firewall and NAT                |
| `docker`                   | Docker installation             |
| `tailscale`                | Tailscale VPN (optional)        |
| `drs`, `config`            | DRS environment                 |
| `journald`                 | journald configuration          |
| `cyclonedds`               | CycloneDDS configuration        |
| `drs_api_service`          | DRS API service                 |
| `drs_ros2_bridge_service`  | ROS 2 bridge service            |
| `drs_dashboard_service`    | Dashboard service               |

### Skip specific roles

```bash
# Skip network configuration
./drs-setup.sh --skip-tags netplan

# Skip time synchronization
./drs-setup.sh --skip-tags ptp
```

## Configuration

### Global Variables

Edit `inventory/group_vars/all.yaml` to configure:

- DRS IDs (sensing system, module)
- ROS 2 domain ID
- CycloneDDS parameters
- Data transfer settings
- Time synchronization settings

### Host-specific Variables

Host configurations are stored in:

- `inventory/host_vars/ecu0.yaml` — ECU0 (sensing module)
- `inventory/host_vars/ecu1.yaml` — ECU1 (sensing module)
- `inventory/host_vars/raspi.yaml` — Raspberry Pi (control module)

These files contain network interface configs, PTP settings, NAS addresses, and per-host feature flags.

## Role Execution Order

### Sensing module (`drs-sensing.yaml`)

1. `cleanup` — pre-installation cleanup
2. `drs_config` — DRS environment variables
3. `ssd_mount` — internal SSD
4. `jetpack` — NVIDIA Jetpack
5. `jetson` — Jetson system tuning
6. `journald` — log configuration
7. `netplan` — network interfaces
8. `can_interface` — CAN (ECU0 only)
9. `iptables` — firewall
10. `ptp` — PTP time sync
11. `ntp_server` — NTP server (opt-in)
12. `ros2` — ROS 2
13. `anvil_docker` — Docker
14. `cyclonedds` — middleware
15. `drs` — DRS build/install
16. `drs_recorder_service` / `drs_sensor_service` / `drs_api_service` — core services
17. `drs_ros2_bridge_service` / `drs_dashboard_service` — optional services
18. `tier4_hdr_camera_driver` / `c2_readout_delay_setter` / `sensor_trigger` — hardware drivers
19. `external_ssd_mount` / `nas_mount` / `drs_transfer` — storage and data transfer
20. `extra_apps` — additional utilities

### Control module (`drs-control.yaml`)

1. `raspi_time_sync` — time synchronization
2. `raspi_network` — network interfaces (VLAN)
3. `raspi_hostapd` — WiFi AP
4. `raspi_dhcp` — DHCP server
5. `raspi_iptables` — firewall/NAT
6. `docker` — container runtime
7. `tailscale` — VPN (optional)
8. `drs_config` / `journald` / `cyclonedds` — DRS environment
9. `drs_api_service` / `drs_ros2_bridge_service` / `drs_dashboard_service` — DRS services

## Prerequisites

The script automatically installs required dependencies:

- `pipx` (via apt)
- `ansible-core` (via pipx)
- Ansible Galaxy collections: `ansible.posix`, `community.general`

You only need:

- Python 3 (pre-installed on supported platforms)
- `sudo` access for system configuration

## Troubleshooting

### Hostname must match exactly

The script requires the hostname to be exactly `ecu0`, `ecu1`, or `raspi`. To fix:

```bash
sudo hostnamectl set-hostname ecu0  # or ecu1 / raspi
```

Or override at runtime:

```bash
DRS_TARGET_HOST=ecu0 ./drs-setup.sh
```

### Check service status

```bash
# Sensing module (ECU0/ECU1)
systemctl status drs-recorder.service
systemctl status drs-sensor.service
systemctl status drs-api.service

# Control module (Raspberry Pi)
systemctl status drs-api.service
systemctl status drs-dashboard.service
systemctl status drs-ros2-bridge.service
systemctl status hostapd.service
```

### View logs

```bash
journalctl -u drs-recorder.service -f
journalctl -u drs-sensor.service -f
journalctl -u drs-api.service -f
```

### Rerun specific roles

```bash
ansible-playbook -i inventory/localhost.yaml -K -e "target_host=ecu0" drs-sensing.yaml --tags <tag>
```
