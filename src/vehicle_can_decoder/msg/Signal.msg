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
