# object label
string label

# object index
int32 object_index

# object label index
int16 class_index

# object confidence (0..1)
float32 confidence

# object 3D pose
geometry_msgs/Pose pose

# 2D bounding box in pixel space
bw_yolo/UVBoundingBox bounding_box_2d

# 3D bounding box in camera world frame
bw_yolo/XYZBoundingBox bounding_box_3d
