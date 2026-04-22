# A single decoded and optionally transformed CAN signal.
#
# Name and unit are NOT carried per-frame. Resolve them from the schema topic:
#   /vehicle/schema  [vehicle_can_decoder/VehicleSchema]
# Lookup: signal_table[name_id-1] → name and unit_id;
#         unit_table[unit_id-1]   → unit string

# Index into VehicleSchema.signal_table (1-indexed; 0 = unknown/unregistered).
uint16 name_id

# Decoded value after applying the configured transform expression.
float32 value

# Signal status.
uint8 STATUS_OK      = 0  # Value is fresh and valid
uint8 STATUS_TIMEOUT = 1  # CAN message has not arrived within timeout_ms
uint8 STATUS_ERROR   = 2  # Decode or transform error occurred
uint8 STATUS_INITIAL = 3  # Signal defined but not yet received
uint8 status

# Hardware timestamp from the CAN frame (seconds since epoch).
# Zero if the CAN interface does not provide hardware timestamps.
float64 timestamp_can
