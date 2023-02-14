import math
from bw_tools.robot_state import Pose2d
from matplotlib import pyplot as plt


def draw_pose(pose, name="", length=0.15, color=(1.0, 0.0, 0.0)):
    heading = pose.theta
    dx = length * math.cos(heading)
    dy = length * math.sin(heading)
    plt.arrow(pose.x, pose.y, dx, dy, color=color)
    plt.plot(pose.x, pose.y, '.', label=name, color=color)


def main():
    # path = "data/successful_run.txt"
    path = "data/weird_run.txt"
    with open(path) as file:
        contents = file.read()
    data = []
    for line in contents.splitlines():
        row = [float(x) for x in line[1:-1].split(",")]
        data.append(Pose2d(*row))
    
    goal = data.pop(0)
    draw_pose(goal, "goal", color=(0.0, 1.0, 0.0))
    for pose in data:
        draw_pose(pose, "robot", color=(1.0, 0.0, 0.0))
    plt.show()


main()
    