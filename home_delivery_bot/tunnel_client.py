import time
import asyncio
import logging
from lib.tunnel.serial.client import TunnelSerialClient
from lib.tunnel.result import PacketResult


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
        self.protocol.use_double_precision = False
        self.start_time = time.monotonic()  # timer for ping

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

    def get_time(self):
        """Get the time since __init__ was called"""
        return time.monotonic() - self.start_time

    def write_ping(self):
        """Write a ping message. Called externally"""
        self.write("ping", self.get_time())

    def set_motor_enable(self, state):
        self.write_handshake("motor_enable", state, write_interval=1.0, timeout=10.0)

    async def get_motor_enable(self):
        result = await self.get("is_motor_enabled", timeout=2.0)
        return bool(result.get_int())

    def set_left_motor_velocity(self, velocity: int):
        self.write("l", int(velocity))

    def set_right_motor_velocity(self, velocity: int):
        self.write("r", int(velocity))

    def stop(self):
        self.set_motor_enable(False)
        self.set_left_motor_velocity(0)
        self.set_right_motor_velocity(0)
        super().stop()
