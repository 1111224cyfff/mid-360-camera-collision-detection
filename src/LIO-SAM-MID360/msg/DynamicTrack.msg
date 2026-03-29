uint8 STATE_TENTATIVE=0
uint8 STATE_CONFIRMED=1

int32 track_id
uint8 state
geometry_msgs/Pose pose
geometry_msgs/Vector3 velocity
float32 speed
float32 age_sec
float32 time_since_update_sec
int32 misses