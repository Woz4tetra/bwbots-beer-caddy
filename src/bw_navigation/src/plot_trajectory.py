import time

from matplotlib import pyplot as plt

from bw_navigation.bw_drive_train_optimizer import BwDriveTrainOptimizer
from bw_navigation.chassis_optimizer import ChassisOptimizer
from bw_tools.robot_state import Pose2dStamped
from bw_tools.structs.go_to_goal import GoToPoseGoal


def main():
    optimizer = ChassisOptimizer()
    # optimizer = BwDriveTrainOptimizer()

    goal = GoToPoseGoal.from_xyt("map", 0.0, 1.0, 0.0)
    goal.reference_linear_speed = 0.1
    goal.linear_max_accel = 0.1
    goal.theta_max_accel = 0.5

    robot_state = Pose2dStamped.from_xyt(0.0, 0.0, 0.0, frame_id="map")

    t0 = time.time()
    if optimizer.solve(goal, robot_state):
        print("Success")
    else:
        print("Failure")
        return
    t1 = time.time()
    print("Solve took %0.4f seconds" % (t1 - t0))

    result = optimizer.solve_result

    x = result.states[:, 0]
    y = result.states[:, 1]
    print(x)
    print(y)

    vx = result.states[-1, 3]
    vt = result.states[-1, 5]
    print(vx)
    print(vt)

    fig = plt.figure(1)
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(x, y, '.-', label='path')
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    limit = max(abs(xlim[0]), abs(ylim[0]), abs(xlim[1]), abs(ylim[1]))
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    plt.plot(x, y)
    plt.show()


if __name__ == "__main__":
    main()
