import time
import math
import asyncio
import logging
import aiopubsub
import numpy as np
from beer_caddy.messages import BwModuleStates, Odometry


class ChassisKinematics:
    def __init__(self, logger: logging.Logger, hub: aiopubsub.Hub) -> None:
        self.logger = logger
        self.tunnel_sub = aiopubsub.Subscriber(hub, 'kinematics-tunnel')

        self.width = 0.115  # meters
        self.length = 0.160  # meters
        self.armature = 0.037  # meters

        self.locations = [
            [-self.length / 2.0, self.width / 2.0],  # module 1, channel 0, back left
            [-self.length / 2.0, -self.width / 2.0],  # module 2, channel 1, back right
            [self.length / 2.0, self.width / 2.0],  # module 3, channel 2, front left
            [self.length / 2.0, -self.width / 2.0],  # module 4, channel 3, front right
        ]

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

    async def start_listeners(self):
        self.tunnel_sub.add_async_listener(aiopubsub.Key('tunnel', 'module'), self.on_module)
        self.tunnel_sub.add_async_listener(aiopubsub.Key('tunnel', 'odom'), self.on_odom)
        await asyncio.sleep(0.0)

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
        # self.logger.info(f"calc odom: {self.odom.x: 0.4f}, {self.odom.y: 0.4f}, {self.odom.theta: 0.4f}, {self.odom.vx: 0.4f}, {self.odom.vy: 0.4f}, {self.odom.vt: 0.4f}")
        self.logger.info(f"calc odom: {self.odom.vx: 0.4f}, {self.odom.vy: 0.4f}, {self.odom.vt: 0.4f}")
        return self.odom

    async def on_module(self, key: aiopubsub.Key, states: BwModuleStates):
        self.to_chassis(states)
        await asyncio.sleep(0.0)
    
    async def on_odom(self, key: aiopubsub.Key, odom: Odometry):
        self.logger.info(f"recv odom: {odom.vx: 0.4f}, {odom.vy: 0.4f}, {odom.vt: 0.4f}")

