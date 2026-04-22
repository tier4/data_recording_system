# Vehicle CAN signal schema published on /vehicle/schema.
#
# Provides the lookup tables needed to decode Signal.name_id and unit_id.
# Published once at startup with QoS transient_local so late subscribers
# receive it immediately on connection.
#
# Usage:
#   signal_name = signal_table[Signal.name_id - 1].name
#   unit_string = unit_table[signal_table[Signal.name_id - 1].unit_id - 1]

std_msgs/Header header

# Vehicle identifier matching SignalGroup.vehicle_id.
string vehicle_id

# Semantic version of this schema (e.g. "1.0.0").
# major: any name_id or unit_id changes (breaks bag compatibility)
# minor: new signals or units added (backward compatible)
# patch: documentation-only changes
string schema_version

# Signal lookup table, ordered by id (1-indexed).
# signal_table[name_id - 1] resolves a Signal.name_id.
vehicle_can_decoder/SignalEntry[] signal_table

# Unit lookup table, ordered by id (1-indexed).
# unit_table[unit_id - 1] resolves a SignalEntry.unit_id.
string[] unit_table
