import time
import math
import asyncio
import logging
from lib.tunnel.serial.client import TunnelSerialClient
from lib.tunnel.result import PacketResult
from lib.recursive_namespace import RecursiveNamespace


class RobotTunnelClient(TunnelSerialClient):
    """Wrapper class for TunnelSerialClient adds functionality specific to the hopper swapper"""

    def __init__(self, logger: logging.Logger, path, baud=1000000):
        """
        :param logger: logger object generated from LoggerManager
        :param path: path to arduino device. ex: "/dev/ttyACM0"
        :param baud: communication rate. Must match value defined on the arduino
        """
        super().__init__(path, baud, debug=False)
        self.logger = logger
        self.protocol.use_double_precision = True
        self.start_time = time.monotonic()  # timer for ping
        self.encoder_states = RecursiveNamespace(
            left_ticks=0.0,
            right_ticks=0.0,
            left_speed_ticks=0.0,
            right_speed_ticks=0.0,
        )
        self.imu_states = RecursiveNamespace(
            x=0.0,
            y=0.0,
            z=0.0,
        )
        self.motors_states = RecursiveNamespace(
            left=0,
            right=0,
        )

    async def packet_callback(self, result: PacketResult):
        """
        Callback for when a new packet is received
        :param result: PacketResult object containing data within the packet
        :return: None
        """
        if result.category == "ping":
            sent_time = result.get_float()
            current_time = self.get_time()
            ping = current_time - sent_time
            self.logger.info("Ping: %0.5f (current: %0.5f, recv: %0.5f)" % (ping, current_time, sent_time))
            await asyncio.sleep(0.0)
        elif result.category == "enc":
            self.encoder_states.left_ticks = result.get_int()
            self.encoder_states.right_ticks = result.get_int()
            self.encoder_states.left_speed_ticks = result.get_float()
            self.encoder_states.right_speed_ticks = result.get_float()
        elif result.category == "imu":
            self.imu_states.x = result.get_float()
            self.imu_states.y = result.get_float()
            self.imu_states.z = result.get_float()
        elif result.category == "motors":
            self.motors_states.left = result.get_int()
            self.motors_states.right = result.get_int()

    def get_time(self):
        """Get the time since __init__ was called"""
        return time.monotonic() - self.start_time

    def write_ping(self):
        """Write a ping message. Called externally"""
        self.write("ping", "f", self.get_time())

    def set_motor_enable(self, state):
        self.write_handshake("motor_enable", "b", state, write_interval=1.0, timeout=10.0)

    async def get_motor_enable(self):
        result = await self.get("is_motor_enabled", "", timeout=2.0)
        return result.get_bool()

    def set_left_motor_velocity(self, velocity: int):
        self.write("l", "d", int(velocity))

    def set_right_motor_velocity(self, velocity: int):
        self.write("r", "d", int(velocity))
    
    async def set_balance_config(self, config: RecursiveNamespace):
        self.logger.info("Setting balance config: %s" % str(config))
        setpoint = math.radians(config.setpoint)
        result = await self.get("balance", "eee", setpoint, config.kp, config.kd, timeout=2.0)
        assert result.get_double() == setpoint
        assert result.get_double() == config.kp
        assert result.get_double() == config.kd
        self.logger.info("Controller config set")

    async def enable_balance(self, state: bool):
        result = await self.get("balen", "b", state, timeout=2.0)
        assert result.get_bool() == state
        self.logger.info("Controller enable set to %s" % state)

    def stop(self):
        self.set_motor_enable(False)
        self.set_left_motor_velocity(0)
        self.set_right_motor_velocity(0)
        super().stop()
