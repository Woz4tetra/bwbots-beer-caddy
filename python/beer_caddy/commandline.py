import math
import asyncio

from aiocmd import aiocmd

from lib.exceptions import *


class RobotCLI(aiocmd.PromptToolkitCmd):
    def __init__(self, mysession):
        super().__init__()
        self.mysession = mysession
        self.logger = self.mysession.logger
        self.tunnel = self.mysession.tunnel

    async def do_enable(self, state):
        """Set motor enable state"""
        state = bool(int(state))
        self.logger.info("Motor enable to %s" % repr(state))
        await self.enable_motors(state)
    
    async def enable_motors(self, state):
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

    async def do_d(self, vx, vy, vt):
        """Move left motor at velocity"""
        state = await self.is_motor_enabled()
        if not state:
            self.logger.warning("Motors not enabled! Not sending velocities")
            return
        self.tunnel.drive(vx, vy, vt)

    async def do_s(self):
        """Stop motors"""
        state = await self.is_motor_enabled()
        if not state:
            self.logger.warning("Can't set motor. Motors aren't enabled")
            return
        self.tunnel.drive(0.0, 0.0, 0.0)

    def _on_close(self):
        raise SessionFinishedException
