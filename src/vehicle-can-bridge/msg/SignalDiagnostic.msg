# Health and diagnostic information for the vehicle_can_conv node.
# Published at a low rate (default 1 Hz) on /vehicle/diagnostics.

std_msgs/Header header

# Vehicle identifier from the vehicle_id parameter.
string vehicle_id

# CAN interface name (e.g. "can0", "vcan0").
string can_interface

# Cumulative counters since node start.
uint64 frames_received   # Total CAN frames read from the socket
uint64 frames_decoded    # Frames successfully matched to a DBC message
uint64 frames_unknown    # Frames with no matching DBC message
uint64 decode_errors     # Frames matched but failed to decode signals

# Names of signals currently in STATUS_TIMEOUT state.
string[] timed_out_signals
