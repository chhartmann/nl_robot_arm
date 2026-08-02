# A known object in the world with its ground-truth pose.
string id                       # e.g. "red_cube"
string kind                     # cube | box | table | other
geometry_msgs/PoseStamped pose  # in world/base frame
float32[3] size                 # bounding box (m), x y z
bool graspable
