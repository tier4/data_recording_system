# TODOS

## Infrastructure (drs_transfer)

### Add TimeoutSec to drs-transfer service to prevent NFS hard-mount deadlock

**What:** Add `TimeoutSec=3600` to `ansible/roles/drs_transfer/templates/drs-transfer.service.j2`.

**Why:** Without a timeout, if `/mnt/nas/data` is mounted with `hard` NFS options and becomes unreachable, the `ls "$dest"` automount trigger in `drs_transfer.sh` blocks forever. The service hangs indefinitely, holds the lock file, and every subsequent timer tick logs "Already running" and exits with failure. Data accumulates on the SSD with no transfers and no alert beyond systemd failure logs.

**Context:** Surfaced during adversarial review of the `refactor/rename-sync-to-x-to-drs-transfer` PR. Fix is a one-liner in the `[Service]` section. Choose a timeout that is safely larger than the worst-case full-SSD transfer time (suggest 3600s = 1 hour as a starting point).

**Effort:** S
**Priority:** P1
**Depends on:** None

---

### Quarantine permanently-failing mcap files instead of blocking all transfers

**What:** In `ansible/roles/drs_transfer/templates/drs_transfer.sh.j2`, move files that fail rsync to a `.failed/` subdirectory under `SOURCE_DIR` rather than setting `transfer_failed=1` and aborting the entire run.

**Why:** If a single mcap file has a corrupt inode or revoked permissions, rsync fails on it every invocation. The current code sets `transfer_failed=1` and calls `error_exit`, so every future run exits non-zero even though all other files transfer successfully. The broken file blocks the schedule permanently with no recovery path.

**Context:** Surfaced during adversarial review of the `refactor/rename-sync-to-x-to-drs-transfer` PR. Consider moving the failed file to `$SOURCE_DIR/.failed/$(date +%Y%m%d)/` with a `.reason` sidecar file containing the rsync exit code and timestamp. The `.failed/` directory can then be inspected and cleared by operators without blocking normal transfers.

**Effort:** M
**Priority:** P2
**Depends on:** None

---

### Gate cleanup role drs-transfer tasks behind an opt-in variable

**What:** In `ansible/roles/cleanup/tasks/main.yml`, add `when: drs_transfer_cleanup | default(false)` to the three drs-transfer cleanup tasks (stop timer/service, remove systemd files, remove drs_transfer directory).

**Why:** The cleanup tasks share the `[external_storage]` tag with the install role. Every full Ansible run stops the timer, removes the units and script, then reinstalls them. Any rsync in progress when the timer is stopped is interrupted mid-file, leaving a partial (unlabelled) file at the destination.

**Context:** Surfaced during adversarial review of the `refactor/rename-sync-to-x-to-drs-transfer` PR. The opt-in variable pattern (`when: drs_transfer_cleanup | default(false)`) is already used by similar roles in this project. Operators wanting to fully uninstall the service would pass `-e drs_transfer_cleanup=true` to ansible-playbook.

**Effort:** S
**Priority:** P2
**Depends on:** None

## Completed
