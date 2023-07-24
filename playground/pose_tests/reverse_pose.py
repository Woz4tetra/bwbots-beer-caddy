import math
from bw_tools.robot_state import Pose2d

def reverse(goal_pose, current_pose):
    error = goal_pose.relative_to(current_pose)
    new_error = error.rotate_by(math.pi)
    new_pose = Pose2d.from_state(current_pose)
    new_pose.theta = Pose2d.normalize_theta(current_pose.theta + math.pi)
    new_goal = current_pose.transform_by(new_error)
    return new_goal, new_pose

def main():
    goal_pose = Pose2d(-1.0, 0.0, 0.0)
    current_pose = Pose2d(0.0, 0.0, 0.0)

    new_goal_pose, new_current_pose = reverse(goal_pose, current_pose)
    print(new_goal_pose)
    print(new_current_pose)


main()
