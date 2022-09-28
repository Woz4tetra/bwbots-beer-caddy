import time
import math
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
        self.protocol.use_double_precision = True
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
        self.write("ping", "f", self.get_time())

    def set_motor_enable(self, state):
        self.write_handshake("en", "b", state, write_interval=1.0, timeout=10.0)

    async def get_motor_enable(self):
        result = await self.get("?en", "", timeout=2.0)
        return result.get_bool()

    def drive(self, vx: float, vy: float, vt: float):
        self.write("d", "fff", float(vx), float(vy), float(vt))

    def stop(self):
        self.set_motor_enable(False)
        self.drive(0.0, 0.0, 0.0)
        super().stop()
