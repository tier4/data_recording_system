# A single decoded and optionally transformed CAN signal.

# Signal name: DBC signal name, or the alias defined in YAML config.
string name

# Decoded value after applying the configured transform expression.
float64 value

# Raw decoded value before any transform (direct DBC physical value).
float64 raw_value

# Physical unit string (e.g. "m/s", "rad", "degC", "%").
# Set from the YAML config transforms section; empty if not configured.
string unit

# Source CAN message ID (as received on the bus).
uint32 can_id

# Signal status.
uint8 STATUS_OK      = 0  # Value is fresh and valid
uint8 STATUS_TIMEOUT = 1  # CAN message has not arrived within timeout_ms
uint8 STATUS_ERROR   = 2  # Decode or transform error occurred
uint8 STATUS_INITIAL = 3  # Signal defined but not yet received
uint8 status

# Hardware timestamp from the CAN frame (seconds since epoch).
# Zero if the CAN interface does not provide hardware timestamps.
float64 timestamp_can
