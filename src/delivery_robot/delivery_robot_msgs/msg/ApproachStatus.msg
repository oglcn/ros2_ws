std_msgs/Header header
int32 target_marker_id
bool armed
bool marker_visible
float32 size_norm        # apparent marker size / arrival threshold; >=1.0 means already close enough
string state             # IDLE, TRACKING, RECOVERING, ARRIVED, FAILED
