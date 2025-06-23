# please don't include /msg, it is not std_msgs/msg/Header
# field name only supports lowercase + '_', msg name must start with Uppercase
std_msgs/Header header

float64 gps_time_week
float64[3] acceleration
float64[3] rotation_rate
uint16 gps_week_number
