# One row in the signal lookup table published on /vehicle/schema.
# Resolves a Signal.name_id to a human-readable name and unit.

# 1-indexed ID matching Signal.name_id.
uint16 id

# Canonical signal name (e.g. "dynamics.speed.longitudinal").
string name

# 1-indexed reference into VehicleSchema.unit_table.
# unit_table[unit_id-1] gives the unit string (e.g. "m/s").
uint16 unit_id
