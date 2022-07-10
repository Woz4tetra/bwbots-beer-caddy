import asyncio

from aiocmd import aiocmd


class RobotCLI(aiocmd.PromptToolkitCmd):
    def __init__(self, tunnel):
        super().__init__()
        self.tunnel = tunnel

    def do_left(self, velocity):
        """Move left motor at velocity"""
        self.tunnel.set_left_motor_velocity(velocity)

    def do_right(self, velocity):
        """Move right motor at velocity"""
        self.tunnel.set_right_motor_velocity(velocity)

    def do_both(self, velocity):
        """Move both motors at velocity"""
        self.tunnel.set_left_motor_velocity(velocity)
        self.tunnel.set_right_motor_velocity(velocity)

    def do_rotate(self, velocity):
        """Move rotate motors at velocity"""
        self.tunnel.set_left_motor_velocity(-velocity)
        self.tunnel.set_right_motor_velocity(velocity)

    def do_stop(self):
        """Stop motors"""
        self.tunnel.set_left_motor_velocity(0.0)
        self.tunnel.set_right_motor_velocity(0.0)
