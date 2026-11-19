# One net-adapter channel as seen by one endpoint; see NetBandwidth.msg.

string topic
bool tx                           # true: this endpoint sends it, false: receives
float64 bytes_per_s
float64 msgs_per_s
uint64 bytes_total
uint64 msgs_total
