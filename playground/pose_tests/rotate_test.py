import math
from typing import Tuple
import numpy as np
from matplotlib import pyplot as plt
from bw_tools.robot_state import Pose2d


def rotate_by_1(x: float, y: float, rotate_theta: float) -> Tuple[float, float]:
    mat = np.array(
        [
            [np.cos(rotate_theta), -np.sin(rotate_theta)],
            [np.sin(rotate_theta), np.cos(rotate_theta)],
        ]
    )
    rotated = mat @ np.array([x, y])
    print(rotated)
    return float(rotated[0]), float(rotated[1])


def rotate_by_2(x, y, theta):
    """
    Apply rotation matrix (defined by theta)
    """
    x1 = x * math.cos(theta) - y * math.sin(theta)
    y1 = x * math.sin(theta) + y * math.cos(theta)
    return x1, y1


def rotate_by_3(vector: np.ndarray, rotate_theta: float) -> np.ndarray:
    mat = np.array(
        [
            [np.cos(rotate_theta), -np.sin(rotate_theta)],
            [np.sin(rotate_theta), np.cos(rotate_theta)],
        ]
    )
    out_vector = np.zeros_like(vector)
    for row_index in range(len(vector)):
        rotated = mat @ vector[row_index]
        out_vector[row_index] = rotated
    return out_vector


def rotate_point_by_3(x: float, y: float, rotate_theta: float) -> Tuple[float, float]:
    result = rotate_by_3(np.array([[x, y]], dtype=np.float64), rotate_theta)[0]
    return (result[0], result[1])

def rotate_by_4(x, y, theta):
    pose = Pose2d(x, y).rotate_by(theta)
    return pose.x, pose.y

pt1 = (-1.0, 1.0)
pt2 = (1.0, 1.0)
theta = -3.0

plt.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]])

results = [pt1, pt2]
for rotate_fn in (rotate_by_1, rotate_by_2, rotate_point_by_3, rotate_by_4):
    rpt1 = rotate_fn(pt1[0], pt1[1], theta)
    rpt2 = rotate_fn(pt2[0], pt2[1], theta)
    results.append(rpt1)
    results.append(rpt2)
    
    plt.plot([rpt1[0], rpt2[0]], [rpt1[1], rpt2[1]])

results = np.array(results)

print(results)
plt.xlim(-2.0, 2.0)
plt.ylim(-2.0, 2.0)
plt.show()

