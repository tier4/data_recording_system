# External SSD Mount Role

Ansible role for automatically mounting external SSDs identified by LABEL.

## Features

- Automatic mounting using systemd automount unit
- Prevents accidental writes before mount (achieved through automount)
- Mount options optimized for SSDs

## Write Prevention Before Mount

By using systemd automount unit:
- Automatically mounts on first access to the mount point
- Mount point doesn't exist when unmounted, preventing writes
- Access is blocked and returns error if SSD is not connected

## Usage

### Basic Usage

```yaml
- hosts: target_hosts
  roles:
    - external_ssd_mount
```

### Customization Example

```yaml
- hosts: target_hosts
  roles:
    - role: external_ssd_mount
      vars:
        ssd_label: COMLOPS
        mount_point: /mnt/external/data
        filesystem_type: ext4
```

## Variables

| Variable | Default Value | Description |
|----------|---------------|-------------|
| `ssd_label` | COMLOPS | SSD LABEL |
| `mount_point` | /mnt/external/data | Mount point path |
| `filesystem_type` | ext4 | Filesystem type |
| `mount_options` | defaults,noatime,nodiratime,errors=remount-ro | Mount options |
| `automount_timeout_idle_sec` | 0 | Auto-unmount timeout when idle (0=disabled) |

## Mount Options Details

- `defaults`: Standard options (rw,suid,dev,exec,auto,nouser,async)
- `noatime`: Don't update access time (extends SSD lifespan)
- `nodiratime`: Don't update directory access time
- `errors=remount-ro`: Remount read-only on errors

## Verification

```bash
# Check mount status
systemctl status mnt-external-data.mount
systemctl status mnt-external-data.automount

# Manual mount/unmount
systemctl start mnt-external-data.mount
systemctl stop mnt-external-data.mount
```