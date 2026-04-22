# A single decoded and optionally transformed CAN signal.

# Signal name: DBC signal name, or the alias defined in YAML config.
string name

# Decoded value after applying the configured transform expression.
float32 value

# Physical unit string (e.g. "m/s", "rad", "degC", "%").
# Set from the YAML config transforms section; empty if not configured.
string unit

# Signal status.
uint8 STATUS_OK      = 0  # Value is fresh and valid
uint8 STATUS_TIMEOUT = 1  # CAN message has not arrived within timeout_ms
uint8 STATUS_ERROR   = 2  # Decode or transform error occurred
uint8 STATUS_INITIAL = 3  # Signal defined but not yet received
uint8 status

# Hardware timestamp from the CAN frame (seconds since epoch).
# Zero if the CAN interface does not provide hardware timestamps.
float64 timestamp_can
