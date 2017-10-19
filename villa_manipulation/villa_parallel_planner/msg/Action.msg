uint8 POSE=0
uint8 LINE=1
uint8 GRIPPER=2
uint8 WORLD=3

uint8 REMOVE

int32 type

# POSE
geometry_msgs/Pose odom_to_hand_pose

# LINE
geometry_msgs/Vector3 axis
float32 distance

# GRIPPER
float32 open_angle
float32 motion_time

# WORLD
int32 op
tmc_msgs/ObjectIdentifier id
