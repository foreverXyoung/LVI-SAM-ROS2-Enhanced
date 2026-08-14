# Cross-component reset event. This is an event notification, not a command
# to alter the LiDAR matcher or factor graph. Receivers honor only the reset
# actions explicitly requested by reset_imu/restart_visual.

uint8 REASON_RELOCALIZATION=0
uint8 REASON_FORCE_RELOCALIZATION=1
uint8 REASON_MAP_CORRECTION=2
uint8 REASON_VINS_FAILURE=3
uint8 REASON_IMU_FAILURE=4
uint8 REASON_IMAGE_STREAM_RESET=5
uint8 REASON_MANUAL=6

std_msgs/Header header

# event_id is unique within the publishing source. reset_id is the
# map/LiDAR continuity generation and is owned by mapOptimization. A visual
# or IMU source may repeat the last observed reset_id while publishing its own
# event; receivers must not deduplicate by reset_id alone.
uint64 event_id
uint64 reset_id
uint8 reason
string source
bool reset_imu
bool restart_visual
string detail
