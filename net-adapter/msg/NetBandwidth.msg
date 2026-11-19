# Traffic of one net-adapter endpoint (robot_endpoint or client_endpoint) over
# the last reporting period.
#
# Bytes are adapter payloads as handed to / received from zenoh, i.e. after
# zstd or the ms136 codec. Zenoh, TCP and IP framing are not included, and
# neither is the latency ping (a few bytes every 250 ms).

builtin_interfaces/Time stamp
float32 period_s                  # length of the period these rates cover

float64 tx_bytes_per_s            # sent by this endpoint
float64 rx_bytes_per_s            # received by this endpoint
uint64 tx_bytes_total             # since the endpoint started
uint64 rx_bytes_total

NetChannelBandwidth[] channels    # per-topic breakdown, fixed order
