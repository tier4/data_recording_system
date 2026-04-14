# A group of decoded CAN signals belonging to the same domain.
#
# Published on:
#   /vehicle/<domain>        - per-domain topic (e.g. /vehicle/chassis)
#   /vehicle/signals/all     - firehose topic containing every decoded signal

std_msgs/Header header

# Domain name as defined in the YAML config (e.g. "chassis", "powertrain").
# Set to "unassigned" for signals whose CAN ID is not in any domain.
string domain

# Vehicle identifier from the vehicle_id parameter.
string vehicle_id

vehicle_can_bridge/Signal[] signals
