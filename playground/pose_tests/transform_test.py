import math
from bw_tools.robot_state import Pose2d
from matplotlib import pyplot as plt


def draw_pose(pose, name="", length=0.15):
    heading = pose.theta
    dx = length * math.cos(heading)
    dy = length * math.sin(heading)
    plt.arrow(pose.x, pose.y, dx, dy)
    plt.plot(pose.x, pose.y, '.', label=name)


pose1 = Pose2d(1.0, 1.0, math.pi / 4)
pose2 = Pose2d(1.5, 0.2, -math.pi / 3)

transform_pose = pose2.transform_by(pose1)
print(transform_pose)

relative_pose = pose2.relative_to(pose1)
print(relative_pose)

undone = transform_pose.relative_to(pose1)

draw_pose(Pose2d(), "origin")
draw_pose(pose1, "1")
draw_pose(pose2, "2")
draw_pose(transform_pose, "transform")
draw_pose(relative_pose, "relative")
draw_pose(undone, "undone")


plt.gca().set_aspect('equal', 'box')
plt.tight_layout()
plt.xlim(-3.0, 3.0)
plt.ylim(-3.0, 3.0)
plt.legend()
plt.show()
