# DRS Sensor Service Role

This Ansible role deploys the DRS sensor nodes as a systemd service (`drs-sensor.service`) that runs `ros2 launch drs_launch drs.launch.xml` on boot.

## Features

- Systemd service management for the DRS sensor launch entry point
- Journald integration for centralized logging
- Automatic restart on failure
- Optional custom parameter directory support
- Real-time scheduling and memory-locking limits for the sensor trigger

## Role Variables

### User Configuration

- `drs_user`: User to run the service (default: `ansible_user` or `nvidia`)
- `drs_group`: Group for the service (default: `ansible_user` or `nvidia`)

### DRS Paths

- `drs_install_dir`: DRS installation directory (default: `/opt/drs/install`)

### Parameter Configuration

- `drs_use_custom_params`: If true, the launch script passes `param_root_dir:={{ drs_param_root_dir }}` to `drs.launch.xml`, and the default parameters shipped with `individual_params` are copied into `drs_param_root_dir` when the directory does not exist yet (default: `true`)
- `drs_param_root_dir`: Root directory for sensor parameters (default: `/opt/drs/config/params`)

### Service Limits

- `sensor_trigger_rtprio`: Real-time priority limit (`LimitRTPRIO`) of the service (default: `85`)
- `sensor_trigger_memlock`: Locked memory limit (`LimitMEMLOCK`) of the service (default: `infinity`)

## What This Role Does

- Creates `/opt/drs/service/drs_sensor/`
- Seeds `drs_param_root_dir` with the default parameters from `individual_params` if missing (only when `drs_use_custom_params` is true)
- Deploys `/opt/drs/service/drs_sensor/launch.sh`, which sources `/opt/drs/config/drs.env`, the ROS 2 Humble setup, and the DRS install setup, then launches `drs_launch drs.launch.xml`
- Deploys, enables, and starts `/etc/systemd/system/drs-sensor.service`

## Example Playbook

```yaml
- hosts: drs_ecus
  roles:
    - role: drs_sensor_service
      vars:
        drs_user: nvidia
        drs_use_custom_params: true
        drs_param_root_dir: /opt/drs/config/params
```

## Service Management

### View logs

```bash
# Real-time logs
journalctl -u drs-sensor -f

# Logs from the last hour
journalctl -u drs-sensor --since "1 hour ago"

# Logs with specific priority
journalctl -u drs-sensor -p err
```

### Service control

```bash
# Check status
systemctl status drs-sensor

# Start/stop/restart
sudo systemctl start drs-sensor
sudo systemctl stop drs-sensor
sudo systemctl restart drs-sensor

# Enable/disable at boot
sudo systemctl enable drs-sensor
sudo systemctl disable drs-sensor
```
