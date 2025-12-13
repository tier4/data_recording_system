# DRS Launch Service Role

This Ansible role configures a systemd service to launch the DRS (Data Recording System) ROS 2 nodes.

## Features

- Systemd service management for ROS 2 launch files
- Full journald integration for centralized logging
- Resource limits and security hardening
- Automatic restart on failure
- Environment variable configuration

## Role Variables

### Required Variables

- `drs_user`: User to run the service (default: ansible_user or 'nvidia')
- `drs_group`: Group for the service (default: ansible_user or 'nvidia')

### Optional Variables

- `drs_install_dir`: DRS installation directory (default: /opt/drs/install)
- `ros_domain_id`: ROS domain ID (default: 0)
- `ros_localhost_only`: Restrict ROS to localhost (default: 0)
- `rmw_implementation`: RMW implementation (default: rmw_cyclonedds_cpp)
- `drs_launch_args`: Additional launch arguments (default: "")
- `drs_device_access`: List of devices to allow access (default: [])
- `drs_extra_env`: Dictionary of extra environment variables (default: {})

### Logging Configuration

- `drs_log_priority`: Log priority level (default: info)
- `drs_log_rate_limit_interval`: Rate limit interval (default: 30s)
- `drs_log_rate_limit_burst`: Rate limit burst (default: 1000)

## Dependencies

- drs
- drs_config
- ros2
- cyclonedds

## Example Playbook

```yaml
- hosts: drs_nodes
  roles:
    - role: drs_launch_service
      vars:
        drs_user: nvidia
        ros_domain_id: 42
        drs_launch_args: "use_sim_time:=false"
        drs_device_access:
          - "/dev/can*"
          - "/dev/ttyUSB*"
        drs_extra_env:
          CUSTOM_VAR: "value"
```

## Service Management

### View logs

```bash
# Real-time logs
journalctl -u drs-launch -f

# Logs from the last hour
journalctl -u drs-launch --since "1 hour ago"

# Logs with specific priority
journalctl -u drs-launch -p err
```

### Service control

```bash
# Check status
systemctl status drs-launch

# Start/stop/restart
sudo systemctl start drs-launch
sudo systemctl stop drs-launch
sudo systemctl restart drs-launch

# Enable/disable at boot
sudo systemctl enable drs-launch
sudo systemctl disable drs-launch
```
