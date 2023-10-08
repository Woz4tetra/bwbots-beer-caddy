import math
from dataclasses import dataclass
from typing import List, Tuple, Union

import numpy as np
import numpy.typing as npt
from numba import njit
from scipy.optimize import LinearConstraint, NonlinearConstraint

SystemStateXArray = npt.NDArray[np.float64]
SystemInputUArray = npt.NDArray[np.float64]
SystemStateX = npt.NDArray[np.float64]
SystemStateXdot = npt.NDArray[np.float64]
SystemInputU = npt.NDArray[np.float64]
SystemConstraints = List[Union[LinearConstraint, NonlinearConstraint]]
SystemOutputY = npt.NDArray[np.float64]
SystemOutputYArray = npt.NDArray[np.float64]

NUM_STATES = 6
NUM_INPUTS = 6


@dataclass(frozen=True)
class ModuleCommand:
    wheel_velocity: float
    azimuth: float


@dataclass(frozen=True)
class FullModulesCommand:
    commands: List[ModuleCommand]


@njit
def vector_rotate_by(vector: np.ndarray, rotate_theta: float) -> np.ndarray:
    """
    Rotate a vector of 2D points by an angle. Assumes continuous memory layout.

    Args:
    vector -- Vector of 2D points.
    rotate_theta -- angle to rotate the points by.

    Returns:
    np.ndarray -- the transformed points. The dimensions will match the vector argument.
    """
    mat = np.array(
        [
            [np.cos(rotate_theta), -np.sin(rotate_theta)],
            [np.sin(rotate_theta), np.cos(rotate_theta)],
        ]
    )
    out_vector: np.ndarray = mat @ vector
    return out_vector


@njit(cache=True)
def compute_ik(module_locations: np.ndarray, armature_length: float, azimuths: np.ndarray) -> np.ndarray:
    ik = []
    for index, (x, y) in enumerate(module_locations):
        azimuth = azimuths[index]
        ik.append([1.0, 0.0, -y - armature_length * np.sin(-azimuth)])
        ik.append([0.0, 1.0, x + armature_length * np.cos(-azimuth)])
    return np.array(ik, dtype=np.float64)


@njit(cache=True)
def compute_chassis_velocities(
    module_locations: np.ndarray, armature_length: float, module_states: np.ndarray
) -> Tuple[float, float, float]:
    module_vector = np.zeros(len(module_locations) * 2, dtype=np.float64)
    azimuths = np.zeros(len(module_locations), dtype=np.float64)
    for index in range(0, len(module_states), 2):
        wheel_velocity = module_states[index]
        azimuth = module_states[index + 1]
        azimuths[index] = azimuth
        vx = wheel_velocity * np.cos(azimuth)
        vy = wheel_velocity * np.sin(azimuth)
        module_vector[index * 2] = vx
        module_vector[index * 2 + 1] = vy
    ik = compute_ik(module_locations, armature_length, azimuths)
    fk = np.linalg.pinv(ik)
    chassis_vector = np.dot(fk, module_vector)
    return chassis_vector[0], chassis_vector[1], chassis_vector[2]


@njit(cache=True)
def compute_next_module(
    module_locations: np.ndarray,
    armature_length: float,
    current_module_states: np.ndarray,
    input_velocity: Tuple[float, float, float],
) -> np.ndarray:
    azimuths = np.zeros(len(module_locations), dtype=np.float64)
    for index in range(0, len(current_module_states), 2):
        azimuth = current_module_states[index + 1]
        azimuths[index] = azimuth
    ik = compute_ik(module_locations, armature_length, azimuths)
    chassis_vector = np.array([input_velocity[0], input_velocity[1], input_velocity[2]])
    next_module_states = np.dot(ik, chassis_vector)
    states = np.zeros(len(module_locations) * 2, dtype=np.float64)
    for index in range(0, len(next_module_states), 2):
        wheel_velocity = next_module_states[index]
        azimuth = next_module_states[index + 1]
        states[index] = wheel_velocity
        states[index + 1] = azimuth
    return states


@njit(cache=True)
def compute_single_module_command(
    x_location: float,
    y_location: float,
    min_radius_of_curvature: float,
    armature_length: float,
    min_angle: float,
    max_angle: float,
    min_wheel_velocity: float,
    max_wheel_velocity: float,
    update_delta_time: float,
    chassis_velocity: Tuple[float, float, float],
) -> Tuple[float, float]:
    dt = update_delta_time
    vx, vy, vt = chassis_velocity
    theta_mag = vt * dt
    v_mag = math.sqrt(vx * vx + vy * vy)
    if theta_mag == 0.0:
        radius_of_curvature = 0.0
    else:
        radius_of_curvature = math.copysign(1.0, vx) * (v_mag * dt) / math.tan(theta_mag)

    if theta_mag == 0.0 or np.isnan(radius_of_curvature) or np.isinf(radius_of_curvature):
        # Strafe regime
        module_vx = vx
        module_vy = vy
        if abs(v_mag) > 0.0:
            # set azimuth if v_mag is > 0.0. Otherwise, use the neutral angle
            azimuth = math.atan2(module_vy, module_vx)
        else:
            azimuth = 0.0
        wheel_velocity = v_mag
    elif abs(radius_of_curvature) < min_radius_of_curvature:
        # Rotate in place regime
        module_vx = vt * -y_location
        module_vy = vt * x_location
        azimuth = math.atan2(module_vy, module_vx)
        wheel_velocity = math.sqrt(module_vx * module_vx + module_vy * module_vy)
        if min_angle < 0.0:
            azimuth += math.pi
            wheel_velocity = -wheel_velocity
    else:
        # Ackermann + strafe regime
        module_angle = math.atan2(x_location, radius_of_curvature - y_location)
        module_radc = x_location / math.sin(module_angle) - armature_length
        module_hypo = math.copysign(1.0, vx) * module_radc / radius_of_curvature

        module_vx = module_hypo * math.cos(module_angle)
        module_vy = module_hypo * math.sin(module_angle)
        azimuth = math.atan2(module_vy, module_vx)
        wheel_velocity = v_mag
        if min_angle < 0.0:
            azimuth += math.pi
            wheel_velocity = -wheel_velocity

    azimuth = max(min_angle, min(max_angle, azimuth))
    wheel_velocity = max(min_wheel_velocity, min(max_wheel_velocity, wheel_velocity))

    return wheel_velocity, azimuth


@njit(cache=True)
def compute_module_commands(
    module_locations: np.ndarray,
    armature_length: float,
    min_radius_of_curvature: float,
    angle_limits: np.ndarray,
    velocity_limits: Tuple[float, float],
    update_delta_time: float,
    chassis_velocity: Tuple[float, float, float],
) -> np.ndarray:
    states = np.zeros(len(module_locations) * 2, dtype=np.float64)
    for index, (x, y) in enumerate(module_locations):
        module_state = compute_single_module_command(
            x,
            y,
            min_radius_of_curvature,
            armature_length,
            angle_limits[index][0],
            angle_limits[index][1],
            velocity_limits[0],
            velocity_limits[1],
            update_delta_time,
            chassis_velocity,
        )
        wheel_velocity, azimuth = module_state
        states[index * 2] = wheel_velocity
        states[index * 2 + 1] = azimuth
    return states


@njit(cache=True)
def module_system_update(state_x: SystemStateX, input_u: SystemInputU) -> SystemStateXdot:
    state_theta = state_x[2]
    input_vx = input_u[0]
    input_vy = input_u[1]

    input_ax = input_u[3]
    input_ay = input_u[4]
    input_at = input_u[5]

    rotated_vel = vector_rotate_by(np.array([[input_vx], [input_vy]]), state_theta)
    rotated_accel = vector_rotate_by(np.array([[input_ax], [input_ay]]), state_theta)

    input_vt = input_u[2]
    return np.array(
        [
            rotated_vel[0, 0],
            rotated_vel[0, 1],
            input_vt,
            rotated_accel[0, 0],
            rotated_accel[0, 1],
            input_at,
        ],
        dtype=np.float64,
    )


def module_system_output(
    input_u: SystemInputU,
    module_locations: np.ndarray,
    armature_length: float,
    min_radius_of_curvature: float,
    angle_limits: np.ndarray,
    velocity_limits: Tuple[float, float],
    update_delta_time: float,
) -> SystemOutputY:
    chassis_velocity = input_u[0], input_u[1], input_u[2]
    module_states = compute_module_commands(
        module_locations,
        armature_length,
        min_radius_of_curvature,
        angle_limits,
        velocity_limits,
        update_delta_time,
        chassis_velocity,
    )
    return module_states


@njit(cache=True)
def strafe_constraint(state_x: SystemStateX, input_u: SystemInputU) -> float:
    angle = math.atan2(input_u[1], input_u[0])
    angle += math.pi
    angle %= math.pi
    angle -= math.pi
    return angle


def warmup():
    points = np.append([np.linspace(5.0, 0.0, 10)], [np.linspace(0.0, 5.0, 10)], axis=0).T
    module_locations = np.array(
        [
            (1.0, 1.0),
            (-1.0, 1.0),
            (-1.0, -1.0),
            (1.0, -1.0),
        ]
    )
    azimuths = np.array([0.0 for _ in range(4)])
    azimuth_limits = np.array([(-1.0, 1.0) for _ in range(4)])
    module_states = np.array([0.0 for _ in range(8)])
    state = np.zeros(6, dtype=np.float64)
    input = np.zeros(6, dtype=np.float64)
    vector_rotate_by(points.T, 0.0)
    compute_ik(module_locations, 0.0, azimuths)
    compute_chassis_velocities(module_locations, 0.0, module_states)
    compute_next_module(module_locations, 0.0, module_states, (0.0, 0.0, 0.0))
    compute_single_module_command(0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.01, (0.0, 0.0, 0.0))
    compute_module_commands(module_locations, 0.0, 1.0, azimuth_limits, (0.0, 1.0), 0.01, (0.0, 0.0, 0.0))
    module_system_update(np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]))
    strafe_constraint(state, input)
