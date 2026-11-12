# Statistics for the current mining session (since the last reset).

# What the robot is doing right now
uint8 STATE_IDLE = 0      # hopper not lowered or raised, or trencher not spinning
uint8 STATE_DIGGING = 1   # hopper lowered and trencher spinning
uint8 STATE_DUMPING = 2   # hopper raised for offload

std_msgs/Header header

uint8 state

float32 session_seconds          # time since the last reset
float32 digging_seconds          # total time spent in STATE_DIGGING
uint32 dump_count                # times the hopper has been raised to dump

float32 trencher_energy_wh       # energy drawn by the trencher, in watt-hours
float32 trencher_peak_current_a  # highest trencher supply current seen, in amps

float32 hopper_fullness          # latest reading from lance/hopper_fullness (0 to 1)
