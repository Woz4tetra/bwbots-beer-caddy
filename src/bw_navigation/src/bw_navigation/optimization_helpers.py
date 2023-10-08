import math
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

NUM_STATES = 8
NUM_INPUTS = 3


@njit(cache=True)
def compute_ik(
    module_locations: List[Tuple[float, float]], armature_length: float, azimuths: List[float]
) -> np.ndarray:
    ik = []
    for index, (x, y) in enumerate(module_locations):
        azimuth = azimuths[index]
        ik.append([1.0, 0.0, -y - armature_length * np.sin(-azimuth)])
        ik.append([0.0, 1.0, x + armature_length * np.cos(-azimuth)])
    return np.array(ik, dtype=np.float64)


@njit(cache=True)
def compute_chassis_velocities(
    module_locations: List[Tuple[float, float]], armature_length: float, module_states: List[float]
) -> Tuple[float, float, float]:
    module_vector = []
    azimuths = []
    for index in range(0, len(module_states), 2):
        wheel_velocity = module_states[index]
        azimuth = module_states[index + 1]
        azimuths.append(azimuth)
        vx = wheel_velocity * np.cos(azimuth)
        vy = wheel_velocity * np.sin(azimuth)
        module_vector.append(vx)
        module_vector.append(vy)
    ik = compute_ik(module_locations, armature_length, azimuths)
    fk = np.linalg.pinv(ik)
    chassis_vector = np.dot(fk, module_vector)
    return chassis_vector[0], chassis_vector[1], chassis_vector[2]


@njit(cache=True)
def compute_module_inputs(
    module_locations: List[Tuple[float, float]],
    armature_length: float,
    current_module_states: List[float],
    input_velocity: Tuple[float, float, float],
) -> List[float]:
    azimuths = []
    for index in range(0, len(current_module_states), 2):
        azimuth = current_module_states[index + 1]
        azimuths.append(azimuth)
    ik = compute_ik(module_locations, armature_length, azimuths)
    chassis_vector = np.array([input_velocity[0], input_velocity[1], input_velocity[2]])
    next_module_states = np.dot(ik, chassis_vector)
    states = []
    for index in range(0, len(next_module_states), 2):
        wheel_velocity = next_module_states[index]
        azimuth = next_module_states[index + 1]
        states.append(wheel_velocity)
        states.append(azimuth)
    return states


@njit(cache=True)
def compute_single_module_command(
    x_location: float,
    y_location: float,
    min_radius_of_curvature: float,
    armature_length: float,
    min_angle: float,
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
    return wheel_velocity, azimuth


@njit(cache=True)
def compute_module_commands(
    module_locations: List[Tuple[float, float]],
    armature_length: float,
    min_radius_of_curvature: float,
    min_angles: List[float],
    update_delta_time: float,
    chassis_velocity: Tuple[float, float, float],
) -> List[Tuple[float, float]]:
    states = []
    for index, (x, y) in enumerate(module_locations):
        module_state = compute_single_module_command(
            x,
            y,
            min_radius_of_curvature,
            armature_length,
            min_angles[index],
            update_delta_time,
            chassis_velocity,
        )
        wheel_velocity, azimuth = module_state
        states.append(wheel_velocity)
        states.append(azimuth)
    return states
