# This expresses constraint defined on end-effector frame broken into:
# 1) Cartesian acceleration vector that consists of linear and angular parts.
# 2) Selected direction in which constraint forces do or do not work
# a bool vector that consists of linear and angular parts.

geometry_msgs/Vector3 linear_acc
geometry_msgs/Vector3 angular_acc

mcr_common_msgs/BoolVector3 linear_force
mcr_common_msgs/BoolVector3 angular_force
