import time
import math
import logging
import numpy as np
from lib.blackboard import Blackboard
from beer_caddy.messages import BwModuleStates, Odometry


class ChassisKinematics:
    def __init__(self, logger: logging.Logger, blackboard: Blackboard) -> None:
        self.logger = logger
        self.blackboard = blackboard
        self.blackboard.add_listener("odom", self.on_odom)
        self.blackboard.add_listener("module", self.on_module)

        self.width = 0.115  # meters
        self.length = 0.160  # meters
        self.armature = 0.037  # meters

        self.locations = [
            [-self.length / 2.0, self.width / 2.0],  # module 1, channel 0, back left
            [-self.length / 2.0, -self.width / 2.0],  # module 2, channel 1, back right
            [self.length / 2.0, self.width / 2.0],  # module 3, channel 2, front left
            [self.length / 2.0, -self.width / 2.0],  # module 4, channel 3, front right
        ]
        self.prev_calc_time = time.time()
        self.prev_recv_time = time.time()

        self.prev_time = time.monotonic()
        self.odom = Odometry()
    
    def get_fk(self, azimuths):
        ik = []
        for index, (x, y) in enumerate(self.locations):
            azimuth = azimuths[index]
            ik.append([1.0, 0.0, -y - self.armature * np.sin(-azimuth)])
            ik.append([0.0, 1.0, x + self.armature * np.cos(-azimuth)])
        ik = np.array(ik)
        fk = np.linalg.pinv(ik)
        return fk

    def dt(self):
        current_time = time.monotonic()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        return dt

    def extrapolate(self, odom, vx, vy, vt) -> Odometry:
        dt = self.dt()
        dx = vx * dt
        dy = vy * dt
        dtheta = vt * dt
        sin_theta = math.sin(dtheta)
        cos_theta = math.cos(dtheta)

        if (abs(dtheta) < 1E-9):
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta
            c = 0.5 * dtheta
        else:
            s = sin_theta / dtheta
            c = (1.0 - cos_theta) / dtheta

        tx = dx * s - dy * c
        ty = dx * c + dy * s

        rotated_tx, rotated_ty = self.rotate_by(tx, ty, odom.theta)
        new_odom = Odometry()
        new_odom.x = odom.x + rotated_tx
        new_odom.y = odom.y + rotated_ty
        new_odom.theta = odom.theta + dtheta
        new_odom.vx = vx
        new_odom.vy = vy
        new_odom.vt = vt

        return new_odom

    def rotate_by(self, x, y, theta):
        x = x * math.cos(theta) - y * math.sin(theta)
        y = x * math.sin(theta) + y * math.cos(theta)
        return x, y

    def to_chassis(self, states: BwModuleStates) -> Odometry:
        module_vector = []
        azimuths = []
        for state in states.states:
            azimuths.append(state.azimuth)
            vx = state.wheel_velocity * np.cos(state.azimuth)
            vy = state.wheel_velocity * np.sin(state.azimuth)
            module_vector.append(vx)
            module_vector.append(vy)
        module_vector = np.array(module_vector)
        chassis_velocities = np.dot(self.get_fk(azimuths), module_vector)

        self.odom = self.extrapolate(
            self.odom,
            chassis_velocities[0],
            chassis_velocities[1],
            chassis_velocities[2]
        )

        # self.logger.info(f"module_vector: " + ", ".join([f"{val:0.4f}" for val in module_vector]))
        # self.logger.info(f"chassis_velocities: " + ", ".join([f"{val:0.4f}" for val in chassis_velocities]))
        
        current_time = time.time()
        if current_time - self.prev_calc_time > 1.0:
            self.prev_calc_time = current_time
            self.logger.info(f"calc odom: {self.odom.x: 0.4f}, {self.odom.y: 0.4f}, {self.odom.theta: 0.4f}, {self.odom.vx: 0.4f}, {self.odom.vy: 0.4f}, {self.odom.vt: 0.4f}")
            # self.logger.info(f"calc odom: {self.odom.vx: 0.4f}, {self.odom.vy: 0.4f}, {self.odom.vt: 0.4f}")

        return self.odom

    def on_module(self, states: BwModuleStates):
        self.to_chassis(states)
    
    def on_odom(self, odom: Odometry):
        current_time = time.time()
        if current_time - self.prev_recv_time > 1.0:
            self.prev_recv_time = current_time
            self.logger.info(f"recv odom: {odom.x: 0.4f}, {odom.y: 0.4f}, {odom.theta: 0.4f}, {odom.vx: 0.4f}, {odom.vy: 0.4f}, {odom.vt: 0.4f}")
            # self.logger.info(f"recv odom: {odom.vx: 0.4f}, {odom.vy: 0.4f}, {odom.vt: 0.4f}")

