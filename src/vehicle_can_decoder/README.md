# vehicle_can_decoder

## ROS2 CAN-to-Topic Abstraction Layer

A config-driven ROS2 Humble C++ node that reads raw SocketCAN frames from a CAN interface, decodes them using DBC files, applies value transformations, and publishes normalized vehicle signals to ROS2 topics.

**Key Feature**: Add new vehicles or signals by editing YAML and DBC files—no recompilation needed.

---

## Overview

vehicle_can_decoder bridges the gap between raw CAN data and ROS2 applications:

```text
can_msgs/Frame topic → DbcDecoder → SignalTransformer → SignalRouter → ROS2 Topics
  (/vehicle/from_can_bus)  (dbcppp)   (exprtk math)     (domains)     (SignalGroup)
                                                                        (std_msgs/Float64)
```

### What It Does

1. **Subscribes** to a `can_msgs/Frame` topic (default: `/vehicle/from_can_bus`)
2. **Decodes signals** using DBC files and the dbcppp library
3. **Transforms values** via user-defined exprtk expressions (e.g., unit conversions, offsets)
4. **Routes by domain** to grouped SignalGroup topics (e.g., `/vehicle/chassis`)
5. **Publishes promoted signals** as std_msgs/Float64 for simple consumers
6. **Monitors health** with diagnostic counters and timeout detection

### Output Topics

- **Signal schema**: `/vehicle/schema` — `VehicleSchema` published at startup and periodically (default: 1 Hz, controlled by `schema_republish_interval_s`) with transient_local QoS; provides name/unit lookup tables for `Signal.name_id`
- **Decoded CAN**: `/vehicle/decoded_can` — all schema-assigned signals in one `SignalGroup` per tick
- **Domain-grouped** (optional): `/vehicle/<domain>` — per-domain `SignalGroup` (enabled by `schema_publish_per_domain: true`)
- **Promoted signals**: `/vehicle/signals/<suffix>` — individual signals as `std_msgs/Float64`
- **Diagnostics**: `/vehicle/diagnostics` — frame counts, timeouts, errors

---

## Quick Start

### 1. Build the Package

```bash
cd ~/ros2_ws  # or wherever your ROS2 workspace is
colcon build --packages-select vehicle_can_decoder
source install/setup.bash
```

### 2. Provide a CAN Frame Source

The node subscribes to a `can_msgs/Frame` topic (default: `/vehicle/from_can_bus`).
You need a publisher feeding CAN frames to that topic. Two common approaches:

**Real hardware** — use `ros2_socketcan` to bridge a physical interface:

```bash
# Example: bridge can0 → /vehicle/from_can_bus
ros2 launch ros2_socketcan socket_can_receiver.launch.xml \
  interface:=can0 topic_name:=/vehicle/from_can_bus
```

**Virtual CAN for testing** — use `vcan` + `ros2_socketcan`:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

ros2 launch ros2_socketcan socket_can_receiver.launch.xml \
  interface:=vcan0 topic_name:=/vehicle/from_can_bus
```

### 3. Create a Vehicle Config File

Copy and adapt the example config:

```bash
cp ~/ros2_ws/src/vehicle_can_decoder/config/example_vehicle.yaml ~/my_vehicle.yaml
```

Edit `~/my_vehicle.yaml`:

- Set `vehicle_id` to your vehicle identifier
- Set `can_topic` to the topic your CAN bridge publishes to (e.g., `/vehicle/from_can_bus`)
- Set `dbc_file` to the absolute path of your DBC file
- Define domains, aliases, and transforms (see [Configuration](#configuration) below)

### 4. Launch the Node

```bash
ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
  config_file:=/absolute/path/to/my_vehicle.yaml
```

Or with a local config:

```bash
ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
  config_file:="$(pwd)/my_vehicle.yaml"
```

When using both a schema YAML and a DBC-specific YAML, pass them together in the launch file
(see `config/vehicle_schema.yaml` and `config/pacmod_v3.yaml` for an example).

### 5. Verify Topics

In another terminal:

```bash
# List published topics
ros2 topic list

# Inspect the signal schema (published at startup and periodically; transient_local QoS)
ros2 topic echo /vehicle/schema

# Listen to domain-grouped signals
ros2 topic echo /vehicle/chassis

# Listen to diagnostics
ros2 topic echo /vehicle/diagnostics
```

---

## Architecture

### Components

| Module                | Purpose                                                                                  | Key Classes                            |
| --------------------- | ---------------------------------------------------------------------------------------- | -------------------------------------- |
| **DbcDecoder**        | Loads DBC files; decodes CAN frames to signal name-value pairs                           | `DbcDecoder`, `RawSignal`              |
| **SignalTransformer** | Compiles and evaluates exprtk math expressions for value transforms                      | `SignalTransformer`, `TransformConfig` |
| **SignalRouter**      | Routes signals to domains; applies signal name aliases                                   | `SignalRouter`, `DomainConfig`         |
| **TimeoutMonitor**    | Tracks signal freshness; detects stale signals                                           | `TimeoutMonitor`                       |
| **CanReader**         | Provides the `CanFrame` struct (used by DbcDecoder and MCAP tooling)                     | `CanFrame`                             |
| **VehicleCanNode**    | ROS2 node: subscribes to `can_msgs/Frame`, orchestrates all components, publishes topics | `VehicleCanNode`                       |

### Signal Flow

1. **`can_msgs/Frame` message arrives** on the subscribed topic
2. **DbcDecoder** looks up the CAN ID in the loaded DBC and decodes signals
3. **SignalRouter** applies aliases and determines the domain
4. **SignalTransformer** applies per-signal math transformations
5. **TimeoutMonitor** checks if the signal is fresh (within `signal_timeout_ms`)
6. **VehicleCanNode** batches signals by domain and publishes to ROS2 topics

### Message Types

**Signal.msg** — A single decoded signal (compact, schema-indexed):

- `name_id` (`uint16`) — 1-indexed key into `VehicleSchema.signal_table`; resolve name and unit via `signal_table[name_id-1]`
- `value` (`float32`) — Transformed value

Name and unit strings are **not** carried per-frame. Resolve them once from the `/vehicle/schema` topic:

```text
signal_name = schema.signal_table[signal.name_id - 1].name
unit_id     = schema.signal_table[signal.name_id - 1].unit_id
unit_string = schema.unit_table[unit_id - 1]
```

**SignalEntry.msg** — One row in the signal lookup table:

- `id` (`uint16`) — 1-indexed ID matching `Signal.name_id`
- `name` (`string`) — Canonical signal name (e.g., `"dynamics.speed.longitudinal"`)
- `unit_id` (`uint16`) — 1-indexed reference into `VehicleSchema.unit_table`

**VehicleSchema.msg** — Published at startup and periodically on `/vehicle/schema` with transient_local QoS (see `schema_republish_interval_s`):

- `header` — Standard ROS2 header
- `vehicle_id` — From config
- `schema_version` (`string`) — Semantic version (major bump = breaking bag change)
- `signal_table[]` — Array of `SignalEntry`; index by `name_id - 1`
- `unit_table[]` (`string[]`) — Unit strings; index by `unit_id - 1`

**SignalGroup.msg** — Grouped signals from one domain:

- `header` — ROS2 standard header with timestamp
- `domain` — Domain name (e.g., `"chassis"`)
- `vehicle_id` — From config
- `signals[]` — Array of `Signal.msg`

**SignalDiagnostic.msg** — Node health (published at `diagnostics_rate_hz`):

- `header` — ROS2 standard header with timestamp
- `vehicle_id` — From config
- `can_topic` — ROS2 topic name providing CAN frames
- `frames_received` — Total `can_msgs/Frame` messages received
- `frames_decoded` — Frames successfully matched to a DBC message
- `frames_unknown` — Frames with no matching DBC message
- `decode_errors` — Frames matched but failed to decode signals
- `timed_out_signals[]` — Names of signals currently flagged as stale by `TimeoutMonitor`

---

## Configuration

Configuration is via YAML. See `/config/example_vehicle.yaml` for a complete example.

### Top-Level Parameters

```yaml
vehicle_can_node:
  ros__parameters:
    # ── Basic Identity ────────────────────────────────────
    vehicle_id: "vehicle_123" # String ID (in all published messages)
    can_topic: "/vehicle/from_can_bus" # Source topic for can_msgs/Frame
    dbc_file: "/path/to/your/vehicle.dbc" # Absolute path to DBC file

    # ── Timing ────────────────────────────────────────────
    signal_timeout_ms: 500 # Mark signal stale after this many ms
    diagnostics_rate_hz: 1.0 # How often to publish diagnostics
    schema_republish_interval_s: 1.0 # How often to re-publish VehicleSchema (ensures each MCAP segment contains the schema)

    # ── Topics ────────────────────────────────────────────
    publish_all_signals: true # Publish firehose topic?
    all_signals_topic: "/vehicle/decoded_can" # Decoded CAN topic name
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
   cp ~/ros2_ws/src/vehicle_can_decoder/config/example_vehicle.yaml \
      ~/vehicle_dbcs/new_vehicle.yaml
   ```

3. **Edit the config file**:

   - Set `vehicle_id` to a unique identifier
   - Set `can_topic` to the ROS2 topic publishing `can_msgs/Frame` (default: `/vehicle/from_can_bus`)
   - Set `dbc_file` to the absolute path of the DBC file
   - List all CAN message IDs from the DBC and group them into domains
   - For each CAN message and signal, add an entry to `transforms` if value conversion is needed
   - Add promoted signals if simple consumers need them

4. **Launch** with the new config:

   ```bash
   ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
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

### 2. Bridge vcan0 to a ROS2 Topic

```bash
# Publish can_msgs/Frame from vcan0
ros2 launch ros2_socketcan socket_can_receiver.launch.xml \
  interface:=vcan0 topic_name:=/vehicle/from_can_bus
```

### 3. Send Test Frames

Use `cansend` (from `can-utils`) while the bridge is running:

```bash
# Send a frame with ID 0x100 and 8 bytes of data
cansend vcan0 100#0102030405060708

# Or use candump to monitor while sending
candump vcan0 &
cansend vcan0 100#1122334455667788
```

### 4. Run Unit Tests

```bash
cd ~/ros2_ws
colcon test --packages-select vehicle_can_decoder --event-handlers console_direct+
```

### 5. Run with Virtual Interface

```bash
# Create a config pointing at the bridged topic
cp ~/ros2_ws/src/vehicle_can_decoder/config/example_vehicle.yaml \
   ~/test_vehicle.yaml

# Edit test_vehicle.yaml and set:
# can_topic: "/vehicle/from_can_bus"
# dbc_file: "/path/to/test.dbc"  (must exist)

ros2 launch vehicle_can_decoder vehicle_can_decoder.launch.py \
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

- **Check the bridge**: Verify `ros2_socketcan` (or your CAN source) is publishing to the topic
- **Verify traffic**: `ros2 topic hz /vehicle/from_can_bus` should show a non-zero rate
- **Check topic name**: `can_topic` in config must match the topic published by the bridge

### Stale signals reported in diagnostics (`timed_out_signals`)

- **Cause**: CAN messages not arriving within `signal_timeout_ms`
- **Fix**: Increase `signal_timeout_ms` or verify CAN traffic is present for those signals

### Decoding schema signal names from Signal.msg

- `Signal.name_id` is 0 when the signal has no schema entry — check that `signal_id_names` in
  your schema YAML covers the signal
- Use `/vehicle/schema` (transient_local) to resolve `name_id` to a name and unit string

### Transform expressions fail to compile

- **Cause**: Syntax error in exprtk expression or reference to undefined variable
- **Fix**: Check `transforms.*.expression` syntax; only `x` (the raw value) is available

### Topics not publishing

- Check node is running: `ros2 node list`
- Check topic names match config: `ros2 topic list`
- Check for errors in node output: `ros2 launch ... --launch-prefix="gdb -ex run --args"`

---

## Performance Notes

- **Event-driven**: The node processes frames as they arrive from the `can_msgs/Frame` subscription; there is no fixed loop rate to tune.
- **Signal batching**: Signals are accumulated per domain within one timer tick before publishing.
- **Timeout monitoring**: Timeout checks occur at the loop rate; resolution is limited by loop period.
- **Transform compilation**: Expressions are compiled once at startup; evaluation is fast.

---

## Building from Source

```bash
# In your ROS2 workspace
cd ~/ros2_ws/src
git clone <this-repo> vehicle_can_decoder
cd ~/ros2_ws

# Build
colcon build --packages-select vehicle_can_decoder

# Source the overlay
source install/setup.bash
```

---

## Offline MCAP Conversion (Building a Converter Tool)

This section describes how to build a standalone tool that converts an MCAP file containing
raw CAN frames (`can_msgs/Frame`) into a new MCAP file containing the abstracted vehicle
signal topics (`vehicle_can_decoder/msg/SignalGroup`), without a running ROS runtime.

### Core Library Reusability

The conversion components are implemented as a **ROS-independent C++ library**
(`vehicle_can_decoder_lib`) and can be linked from any standalone executable.

| Component           | ROS dependency | Role in conversion pipeline             |
| ------------------- | -------------- | --------------------------------------- |
| `DbcDecoder`        | None           | CAN frame → raw signal name-value pairs |
| `SignalTransformer` | None           | Raw value → transformed value + unit    |
| `SignalRouter`      | None           | Signal name aliasing and domain lookup  |
| `CanFrame`          | None           | Input struct (id, data, dlc, timestamp) |
| `VehicleCanNode`    | rclcpp         | **Not reusable** — ROS node only        |

### Conversion Pipeline

The per-frame conversion follows the same path as the live node:

```text
can_msgs/Frame (from MCAP)
    │  CDR deserialize
    ▼
CanFrame { id, data, dlc, timestamp }
    │  DbcDecoder::decode()
    ▼
vector<RawSignal> { name, value, can_id }
    │  SignalRouter::apply_alias()
    │  SignalRouter::domain_for_id()
    │  SignalTransformer::transform()
    ▼
per-domain SignalGroup (vehicle_can_decoder/msg/SignalGroup)
    │  CDR serialize
    ▼
output MCAP (original timestamp preserved)
```

### Code Example

```cpp
#include "vehicle_can_decoder/dbc_decoder.hpp"
#include "vehicle_can_decoder/signal_transformer.hpp"
#include "vehicle_can_decoder/signal_router.hpp"
#include "vehicle_can_decoder/can_reader.hpp"  // for CanFrame struct

using namespace vehicle_can_decoder;

// ── 1. Setup (once at startup) ──────────────────────────────────────────────

DbcDecoder decoder;
if (!decoder.load("/path/to/vehicle.dbc")) {
    throw std::runtime_error("Failed to load DBC");
}

SignalTransformer transformer;
transformer.configure({
    {"VehicleSpeed",   {"x / 3.6",           "m/s"}},
    {"SteeringAngle",  {"x * 3.14159 / 180", "rad"}},
});

SignalRouter router;
std::vector<DomainConfig> domains = {
    {"dynamics", "/vehicle/dynamics", {0x100, 0x101}},
    {"chassis",  "/vehicle/chassis",  {0x200, 0x201}},
};
std::unordered_map<std::string, std::string> aliases = {
    {"VEH_SPD", "VehicleSpeed"},
    {"STR_ANG", "SteeringAngle"},
};
router.configure(domains, aliases);

// ── 2. Per-frame conversion (called for each can_msgs/Frame in the MCAP) ───

// frame.id, frame.data, frame.dlc, frame.timestamp are populated from the
// deserialized can_msgs/Frame message. The timestamp comes directly from
// msg.header.stamp to preserve the original recording time.
auto convert_frame(
    const CanFrame & frame,
    const DbcDecoder & decoder,
    const SignalTransformer & transformer,
    const SignalRouter & router)
    -> std::unordered_map<std::string, std::vector</* Signal */>>
{
    auto raw_signals = decoder.decode(frame.id, frame.data, frame.dlc);
    if (!raw_signals) return {};  // unknown CAN ID — skip

    std::unordered_map<std::string, std::vector</* Signal */>> by_domain;

    for (const RawSignal & raw : *raw_signals) {
        const std::string compound_key =
            "CAN" + std::to_string(frame.id) + "_" + raw.name;
        const std::string & aliased =
            (router.apply_alias(compound_key) != compound_key)
                ? router.apply_alias(compound_key)
                : router.apply_alias(raw.name);

        const std::string & domain = router.domain_for_id(frame.id);
        const TransformResult tr = transformer.transform(aliased, raw.value);

        // Build Signal message and accumulate into the domain group.
        // Serialize to SignalGroup and write to the output MCAP with
        // the original frame.timestamp as the message timestamp.
        by_domain[domain].push_back(/* build Signal from aliased, tr, frame */);
    }
    return by_domain;
}
```

### Recommended MCAP I/O

Use the **`rosbag2_cpp` reader/writer API** for reading and writing MCAP files.
It handles CDR serialization of ROS message types automatically and does not
require `rclcpp::init` or a running ROS executor.

```cpp
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

// Read
rosbag2_cpp::Reader reader;
reader.open("input.mcap");
while (reader.has_next()) {
    auto msg = reader.read_next();
    // msg->topic_name, msg->time_stamp (nanoseconds), msg->serialized_data (CDR)
}

// Write (preserve original timestamps from msg->time_stamp)
rosbag2_cpp::Writer writer;
writer.open("output.mcap");
writer.write(serialized_signal_group_msg, "/vehicle/dynamics", rclcpp::Time{msg->time_stamp});
```

Non-CAN topics in the input MCAP can be copied verbatim by passing their
serialized data directly to the writer without deserialization.

### CMakeLists.txt for the Converter Executable

```cmake
find_package(vehicle_can_decoder REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(can_msgs REQUIRED)

add_executable(mcap_converter src/mcap_converter.cpp)
target_link_libraries(mcap_converter vehicle_can_decoder::vehicle_can_decoder_lib)
ament_target_dependencies(mcap_converter rosbag2_cpp can_msgs)
```

### Timestamp Handling

The converter must preserve the original recording timestamps end-to-end:

| Source                     | How to obtain                                                  |
| -------------------------- | -------------------------------------------------------------- |
| Input CAN frame time       | `rosbag2` message `time_stamp` field (nanoseconds since epoch) |
| `SignalGroup.header.stamp` | Set from the same `time_stamp` — do **not** use wall clock     |
| MCAP write time            | Pass the same `time_stamp` to `writer.write()`                 |

This ensures the output MCAP is fully reproducible regardless of the
processing environment or machine speed.

---

## License

Apache License 2.0

---

## Contact

Maintainer: <maintainer@example.com>
