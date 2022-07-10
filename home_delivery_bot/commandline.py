import sys
import asyncio

from aiocmd import aiocmd

from lib.exceptions import *


class RobotCLI(aiocmd.PromptToolkitCmd):
    def __init__(self, logger, tunnel):
        super().__init__()
        self.logger = logger
        self.tunnel = tunnel

    async def do_enable(self, state):
        """Set motor enable state"""
        state = bool(int(state))
        self.logger.info("Motor enable to %s" % repr(state))
        self.tunnel.set_motor_enable(state)
        result_state = await self.is_motor_enabled()
        if result_state != state:
            self.logger.warning("Motor enable command failed! State did not change")

    async def is_motor_enabled(self):
        try:
            return await self.tunnel.get_motor_enable()
        except asyncio.TimeoutError:
            self.logger.error("Timed out while getting motor enable state")
            return None

    async def do_l(self, velocity):
        """Move left motor at velocity"""
        state = await self.is_motor_enabled()
        if not state:
            self.logger.warning("Can't set motor. Motors aren't enabled")
            return
        velocity = int(velocity)
        self.logger.info("Set left motor to %s" % repr(velocity))
        self.tunnel.set_left_motor_velocity(velocity)

    async def do_r(self, velocity):
        """Move right motor at velocity"""
        state = await self.is_motor_enabled()
        if not state:
            self.logger.warning("Can't set motor. Motors aren't enabled")
            return
        velocity = int(velocity)
        self.logger.info("Set right motor to %s" % repr(velocity))
        self.tunnel.set_right_motor_velocity(velocity)

    async def do_b(self, velocity):
        """Move both motors at velocity"""
        state = await self.is_motor_enabled()
        if not state:
            self.logger.warning("Can't set motor. Motors aren't enabled")
            return
        velocity = int(velocity)
        self.logger.info("Set both motors to %s" % repr(velocity))
        self.tunnel.set_left_motor_velocity(velocity)
        self.tunnel.set_right_motor_velocity(velocity)

    async def do_o(self, velocity):
        """Move rotate motors at velocity"""
        state = await self.is_motor_enabled()
        if not state:
            self.logger.warning("Can't set motor. Motors aren't enabled")
            return
        velocity = int(velocity)
        self.logger.info("Set rotate at %s" % repr(velocity))
        self.tunnel.set_left_motor_velocity(-velocity)
        self.tunnel.set_right_motor_velocity(velocity)

    async def do_s(self):
        """Stop motors"""
        state = await self.is_motor_enabled()
        if not state:
            self.logger.warning("Can't set motor. Motors aren't enabled")
            return
        self.logger.info("Stopping both motors")
        self.tunnel.set_left_motor_velocity(0.0)
        self.tunnel.set_right_motor_velocity(0.0)

    def do_sensors(self):
        self.logger.info("Left encoder: %0.4f ticks, %0.4f t/s" % (self.tunnel.encoder_states.left_ticks, self.tunnel.encoder_states.left_speed_ticks))
        self.logger.info("Right encoder: %0.4f ticks, %0.4f t/s" % (self.tunnel.encoder_states.right_ticks, self.tunnel.encoder_states.right_speed_ticks))
        self.logger.info("Imu: %0.4f X, %0.4f Y, %0.4f Z" % (self.tunnel.imu_states.x, self.tunnel.imu_states.y, self.tunnel.imu_states.z))

    def _on_close(self):
        raise SessionFinishedException
