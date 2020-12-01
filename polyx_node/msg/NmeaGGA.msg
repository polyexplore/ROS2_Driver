std_msgs/Header header

uint8   utc_hour
uint8   utc_minute
uint16  utc_millisec
float64 latitude
float64 longitude
uint8   fix_quality
uint8   n_sv_used
float32 hdop
float64 orthometric_height
float32 geoid_undulation
float32 differential_age
uint16  ref_station_id

## @file NmeaGGA.msg
# @brief message type to publish decoded NMEA GGA message
# @param utc_hour           UTC hour
# @param utc_minute         UTC minute
# @param latitude           Latitude (degrees)
# @param longitude          Longitude (degrees)
# @param fix_quality        Solution status
# @param n_sv_used          Number of satellites used in the solution
# @param hdop               Horizontal dilusion of precision
# @param orthometric_height Height obove the geoid
# @param geoid_undulation   Height of the geoid
# @param differential_age   Age of the differential corrections
# @param ref_station_id     Reference station identification number