# 255 = arbitrary point (no marker docking); otherwise ArUco marker ID
uint8 target_marker_id
# Target position in map frame; used when target_marker_id=255
geometry_msgs/Point destination
# Optional intermediate waypoints in map frame
geometry_msgs/Point[] waypoints
