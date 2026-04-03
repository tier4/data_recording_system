# vehicle_can_bridge

## ROS2 CAN-to-Topic Abstraction Layer

A config-driven ROS2 Humble C++ node that reads raw SocketCAN frames from a CAN interface, decodes them using DBC files, applies value transformations, and publishes normalized vehicle signals to ROS2 topics.

**Key Feature**: Add new vehicles or signals by editing YAML and DBC files—no recompilation needed.

---

## Overview

vehicle_can_bridge bridges the gap between raw CAN data and ROS2 applications:

```text
CAN Interface → DbcDecoder → SignalTransformer → SignalRouter → ROS2 Topics
   (can0)       (dbcppp)     (exprtk math)     (domains)      (SignalGroup)
                                                               (std_msgs/Float64)
```

### What It Does

1. **Reads CAN frames** from a SocketCAN interface (real or virtual)
2. **Decodes signals** using DBC files and the dbcppp library
3. **Transforms values** via user-defined exprtk expressions (e.g., unit conversions, offsets)
4. **Routes by domain** to grouped SignalGroup topics (e.g., `/vehicle/chassis`)
5. **Publishes promoted signals** as std_msgs/Float64 for simple consumers
6. **Monitors health** with diagnostic counters and timeout detection

### Output Topics

- **Domain-grouped**: `/vehicle/<domain>` (e.g., `/vehicle/chassis`) — `vehicle_can_bridge/msg/SignalGroup`
- **Firehose**: `/vehicle/signals/all` — all decoded signals in one message
- **Promoted signals**: `/vehicle/signals/<suffix>` — individual signals as `std_msgs/Float64`
- **Diagnostics**: `/vehicle/diagnostics` — frame counts, timeouts, errors

---

## Quick Start

### 1. Build the Package

```bash
cd ~/ros2_ws  # or wherever your ROS2 workspace is
colcon build --packages-select vehicle_can_bridge
source install/setup.bash
```

### 2. Set Up a Virtual CAN Interface (for testing)

```bash
# Load the vcan kernel module
sudo modprobe vcan

# Create a virtual interface
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Verify it's up
ip link show vcan0
```

### 3. Create a Vehicle Config File

Copy and adapt the example config:

```bash
cp ~/ros2_ws/src/vehicle_can_bridge/config/example_vehicle.yaml ~/my_vehicle.yaml
```

Edit `~/my_vehicle.yaml`:

- Set `vehicle_id` to your vehicle identifier
- Set `can_interface` to `vcan0` (or your real CAN interface)
- Set `dbc_file` to the absolute path of your DBC file
- Define domains, aliases, and transforms (see [Configuration](#configuration) below)

### 4. Launch the Node

```bash
ros2 launch vehicle_can_bridge vehicle_can_bridge.launch.py \
  config_file:=/absolute/path/to/my_vehicle.yaml
```

Or with a local config:

```bash
ros2 launch vehicle_can_bridge vehicle_can_bridge.launch.py \
  config_file:="$(pwd)/my_vehicle.yaml"
```

### 5. Verify Topics

In another terminal:

```bash
# List published topics
ros2 topic list

# Listen to domain-grouped signals
ros2 topic echo /vehicle/chassis

# Listen to diagnostics
ros2 topic echo /vehicle/diagnostics
```

---

## Architecture

### Components

| Module                | Purpose                                                             | Key Classes                            |
| --------------------- | ------------------------------------------------------------------- | -------------------------------------- |
| **DbcDecoder**        | Loads DBC files; decodes CAN frames to signal name-value pairs      | `DbcDecoder`, `RawSignal`              |
| **SignalTransformer** | Compiles and evaluates exprtk math expressions for value transforms | `SignalTransformer`, `TransformConfig` |
| **SignalRouter**      | Routes signals to domains; applies signal name aliases              | `SignalRouter`, `DomainConfig`         |
| **TimeoutMonitor**    | Tracks signal freshness; marks stale signals as `STATUS_TIMEOUT`    | `TimeoutMonitor`                       |
| **CanReader**         | Low-level SocketCAN interface reader                                | `CanReader`, `CanFrame`                |
| **VehicleCanNode**    | ROS2 node orchestrating all components; publishes topics            | `VehicleCanNode`                       |

### Signal Flow

1. **CAN frame arrives** → CanReader extracts it from the socket
2. **DbcDecoder** looks up the CAN ID in the loaded DBC and decodes signals
3. **SignalRouter** applies aliases and determines the domain
4. **SignalTransformer** applies per-signal math transformations
5. **TimeoutMonitor** checks if the signal is fresh (within `signal_timeout_ms`)
6. **VehicleCanNode** batches signals by domain and publishes to ROS2 topics

### Message Types

**Signal.msg** — A single decoded signal:

- `name` — Signal name (after alias substitution)
- `value` — Transformed value
- `raw_value` — Value before transform
- `unit` — Physical unit (e.g., "m/s", "rad")
- `can_id` — Source CAN message ID
- `status` — `STATUS_OK`, `STATUS_TIMEOUT`, `STATUS_ERROR`, or `STATUS_INITIAL`
- `timestamp_can` — Hardware CAN timestamp (if available)

**SignalGroup.msg** — Grouped signals from one domain:

- `header` — ROS2 standard header with timestamp
- `domain` — Domain name (e.g., "chassis")
- `vehicle_id` — From config
- `signals[]` — Array of Signal.msg

**SignalDiagnostic.msg** — Node health:

- `frames_received` — Total CAN frames read
- `frames_decoded` — Frames with known CAN IDs
- `frames_unknown` — Frames with unrecognized CAN IDs
- `decode_errors` — Frames that failed to decode
- `timed_out_signals[]` — Names of signals in `STATUS_TIMEOUT`

---

## Configuration

Configuration is via YAML. See `/config/example_vehicle.yaml` for a complete example.

### Top-Level Parameters

```yaml
vehicle_can_node:
  ros__parameters:
    # ── Basic Identity ────────────────────────────────────
    vehicle_id: "vehicle_123" # String ID (in all published messages)
    can_interface: "can0" # CAN interface name
    dbc_file: "/path/to/your/vehicle.dbc" # Absolute path to DBC file

    # ── Timing ────────────────────────────────────────────
    loop_rate_hz: 100.0 # How often to drain CAN socket & publish
    signal_timeout_ms: 500 # Mark signal stale after this many ms
    diagnostics_rate_hz: 1.0 # How often to publish diagnostics

    # ── Topics ────────────────────────────────────────────
    publish_all_signals: true # Publish firehose topic?
    all_signals_topic: "/vehicle/signals/all" # Firehose topic name
    diagnostics_topic: "/vehicle/diagnostics" # Diagnostics topic name
```

### Domain Configuration

Domains group related CAN IDs into a single ROS2 topic:

```yaml
# List of domain names (required for below to work)
domain_names: ["chassis", "powertrain"]

# Per-domain configuration
domains.chassis.topic: "/vehicle/chassis"
domains.chassis.can_ids: [0x100, 0x101, 0x102] # Hex or decimal

domains.powertrain.topic: "/vehicle/powertrain"
domains.powertrain.can_ids: [512, 513] # Decimal (same as 0x200, 0x201)
```

Signals from CAN IDs not in any domain are assigned to the `"unassigned"` domain.

### Signal Aliases

Rename DBC signal names to semantic names:

```yaml
alias_names: ["RAW_SIGNAL_A", "RAW_SIGNAL_B"]

aliases.RAW_SIGNAL_A: "steering_angle"
aliases.RAW_SIGNAL_B: "wheel_speed_front_left"
```

The alias is used in all downstream processing (transforms, promoted topics, etc.).

### Signal Transforms

Apply math expressions to raw decoded values (unit conversions, scaling, etc.):

```yaml
transform_names: ["VEHICLE_SPEED", "steering_angle"]

transforms.VEHICLE_SPEED.expression: "x / 3.6" # km/h → m/s
transforms.VEHICLE_SPEED.unit: "m/s"

transforms.steering_angle.expression: "x * pi / 180.0" # deg → rad
transforms.steering_angle.unit: "rad"
```

**Transform syntax** (exprtk):

- `x` is the raw DBC physical value
- Math constants: `pi`, `e`
- Operators: `+`, `-`, `*`, `/`, `^` (power), `%` (modulo)
- Functions: `sin()`, `cos()`, `tan()`, `sqrt()`, `abs()`, `min()`, `max()`, etc.
- Examples:
  - `x / 100.0` — scale by 100
  - `x * 9.80665` — convert from g to m/s²
  - `sin(x * pi / 180.0)` — sin of x (in degrees)

If a signal has no transform configured, its raw value is passed through unchanged.

### Promoted Signals

Publish selected signals as `std_msgs/Float64` for simple downstream consumers:

```yaml
promoted_signal_names: ["VEHICLE_SPEED", "steering_angle"]

promoted_signals.VEHICLE_SPEED.topic_suffix: "vehicle_speed"
promoted_signals.steering_angle.topic_suffix: "steering_angle"
```

Topic: `/vehicle/signals/<topic_suffix>` (e.g., `/vehicle/signals/vehicle_speed`)

Values are the transformed values (after applying the signal's transform, if any).

---

## Adding a New Vehicle

1. **Get the DBC file** for the vehicle and place it in a safe location (keep it out of version control if NDA-restricted):

   ```bash
   mkdir -p ~/vehicle_dbcs
   cp /path/to/new_vehicle.dbc ~/vehicle_dbcs/
   ```

2. **Create a vehicle config** by copying and customizing the example:

   ```bash
   cp ~/ros2_ws/src/vehicle_can_bridge/config/example_vehicle.yaml \
      ~/vehicle_dbcs/new_vehicle.yaml
   ```

3. **Edit the config file**:

   - Set `vehicle_id` to a unique identifier
   - Set `can_interface` to the target CAN interface (or `vcan0` for testing)
   - Set `dbc_file` to the absolute path of the DBC file
   - List all CAN message IDs from the DBC and group them into domains
   - For each CAN message and signal, add an entry to `transforms` if value conversion is needed
   - Add promoted signals if simple consumers need them

4. **Launch** with the new config:

   ```bash
   ros2 launch vehicle_can_bridge vehicle_can_bridge.launch.py \
     config_file:=~/vehicle_dbcs/new_vehicle.yaml
   ```

5. **Verify** by listening to topics:

   ```bash
   ros2 topic echo /vehicle/chassis
   ```

---

## Testing with Virtual CAN

The package includes unit and integration tests. To test with a virtual CAN interface:

### 1. Set Up Virtual CAN

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### 2. Send Test Frames

Use `cansend` (from `can-utils`):

```bash
# Send a frame with ID 0x100 and 8 bytes of data
cansend vcan0 100#0102030405060708

# Or use candump to monitor while sending
candump vcan0 &
cansend vcan0 100#1122334455667788
```

### 3. Run Unit Tests

```bash
cd ~/ros2_ws
colcon test --packages-select vehicle_can_bridge --event-handlers console_direct+
```

### 4. Run with Virtual Interface

```bash
# Create a config for vcan0
cp ~/ros2_ws/src/vehicle_can_bridge/config/example_vehicle.yaml \
   ~/test_vehicle.yaml

# Edit test_vehicle.yaml and set:
# can_interface: "vcan0"
# dbc_file: "/path/to/test.dbc"  (must exist)

ros2 launch vehicle_can_bridge vehicle_can_bridge.launch.py \
  config_file:=~/test_vehicle.yaml
```

---

## Dependencies

### Build Dependencies

| Package                       | Purpose                     | Version       |
| ----------------------------- | --------------------------- | ------------- |
| **ament_cmake**               | ROS2 CMake build system     | (ROS2 Humble) |
| **rclcpp**                    | ROS2 C++ client library     | (ROS2 Humble) |
| **std_msgs**                  | Standard ROS2 message types | (ROS2 Humble) |
| **rosidl_default_generators** | ROS2 message generation     | (ROS2 Humble) |

### Runtime Dependencies

| Library    | Purpose                                 | Source                                                                                         |
| ---------- | --------------------------------------- | ---------------------------------------------------------------------------------------------- |
| **dbcppp** | DBC file parsing and CAN frame decoding | Fetched from [GitHub](https://github.com/xR3b0rn/dbcppp.git) (commit pinned in CMakeLists.txt) |
| **exprtk** | Mathematical expression evaluation      | Fetched from [GitHub](https://github.com/ArashPartow/exprtk.git) (v0.0.3)                      |

### System Dependencies

- **libsocketcan** (optional) — For real CAN interface support
- **can-utils** (optional) — Tools for testing (cansend, candump)

### Test Dependencies

| Package               | Purpose                         |
| --------------------- | ------------------------------- |
| **ament_cmake_gtest** | GoogleTest integration for ROS2 |
| **ament_lint_auto**   | Automated code linting          |

---

## DBC File Placement

DBC files are **not committed** to version control (often NDA-restricted). Place them on the system running the node:

```bash
# Create a central location for vehicle DBCs
mkdir -p /opt/vehicle_dbcs

# Copy your DBC files there
cp vehicle_a.dbc /opt/vehicle_dbcs/
cp vehicle_b.dbc /opt/vehicle_dbcs/

# Reference in config with absolute path:
# dbc_file: "/opt/vehicle_dbcs/vehicle_a.dbc"
```

**Important**: Ensure the path in your YAML config is an absolute path and the file exists before launching the node. The node will fail to start if the DBC file cannot be opened.

---

## Troubleshooting

### Node fails to start: "No such file or directory"

- **Cause**: DBC file path in config does not exist or is relative
- **Fix**: Use an absolute path in the `dbc_file` config parameter

### No CAN frames received

- **Check the interface**: `ip link show can0` should show `UP` state
- **Verify traffic**: `candump can0` (in another terminal)
- **Check permissions**: CAN sockets often require root or group membership

### Signals stuck in STATUS_TIMEOUT

- **Cause**: CAN messages not arriving within `signal_timeout_ms`
- **Fix**: Increase `signal_timeout_ms` or verify CAN traffic is present

### Transform expressions fail to compile

- **Cause**: Syntax error in exprtk expression or reference to undefined variable
- **Fix**: Check `transforms.*.expression` syntax; only `x` (the raw value) is available

### Topics not publishing

- Check node is running: `ros2 node list`
- Check topic names match config: `ros2 topic list`
- Check for errors in node output: `ros2 launch ... --launch-prefix="gdb -ex run --args"`

---

## Performance Notes

- **Loop rate**: Default 100 Hz is typical for vehicle CAN. Adjust `loop_rate_hz` for your needs.
- **Signal batching**: Signals are accumulated per domain within one timer tick before publishing.
- **Timeout monitoring**: Timeout checks occur at the loop rate; resolution is limited by loop period.
- **Transform compilation**: Expressions are compiled once at startup; evaluation is fast.

---

## Building from Source

```bash
# In your ROS2 workspace
cd ~/ros2_ws/src
git clone <this-repo> vehicle_can_bridge
cd ~/ros2_ws

# Build
colcon build --packages-select vehicle_can_bridge

# Source the overlay
source install/setup.bash
```

---

## License

Apache License 2.0

---

## Contact

Maintainer: <maintainer@example.com>
