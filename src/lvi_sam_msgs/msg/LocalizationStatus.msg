# Stable, observational localization status for the system-level state machine.
#
# Phase 1 deliberately exposes the existing LVI-SAM state without changing the
# point-cloud matcher, factor graph, TF publication, or odometry decisions.

uint8 STATE_UNKNOWN=255
uint8 WAITING_FOR_SENSORS=0
uint8 RELOCALIZING=1
uint8 VERIFYING=2
uint8 TRACKING=3
uint8 DEGRADED=4
uint8 LOST=5
uint8 ERROR=6
uint8 MAPPING=7

uint8 MODE_MAPPING=0
uint8 MODE_LOCALIZATION=1

std_msgs/Header header

uint8 state
uint8 previous_state
uint8 mode
string state_name
string previous_state_name
string reason

bool pose_valid
bool odometry_valid
bool sensors_ready
bool map_ready
bool relocalization_active
bool quality_degraded

# Phase 1 leaves quality metrics unknown. Consumers must not treat these as
# matcher scores until a later state-machine phase defines their computation.
float32 match_score
float32 confidence
float32 lidar_age
float32 imu_age
float32 odometry_age

uint32 consecutive_successes
uint32 consecutive_failures
uint32 relocalization_attempts
uint64 transition_sequence
uint32 loss_count

builtin_interfaces/Time last_valid_pose_stamp
builtin_interfaces/Time state_enter_stamp
builtin_interfaces/Time last_lost_stamp
