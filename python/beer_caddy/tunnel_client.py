import time
import asyncio
import logging
from typing import Optional
import aiopubsub
from lib.tunnel.serial.client import TunnelSerialClient
from lib.tunnel.result import PacketResult
from beer_caddy.messages import ModuleState, Odometry, BwModuleStates


class RobotTunnelClient(TunnelSerialClient):
    """Wrapper class for TunnelSerialClient adds functionality specific to the hopper swapper"""

    def __init__(self, logger: logging.Logger, hub: aiopubsub.Hub, path, baud=1000000):
        """
        :param logger: logger object generated from LoggerManager
        :param path: path to arduino device. ex: "/dev/ttyACM0"
        :param baud: communication rate. Must match value defined on the arduino
        """
        super().__init__(path, baud, debug=False)
        self.hub = hub
        self.logger = logger
        self.protocol.use_double_precision = True
        self.start_time = time.monotonic()  # timer for ping
        self.odom_state = Odometry()
        self.module_states = BwModuleStates()

        self.parsed_pub = aiopubsub.Publisher(self.hub, prefix=aiopubsub.Key('tunnel'))
    
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
        elif result.category == "od":
            x = result.get_double()
            y = result.get_double()
            theta = result.get_double()
            vx = result.get_float()
            vy = result.get_float()
            vt = result.get_float()

            self.odom_state.timestamp = time.monotonic()
            self.odom_state.x = x
            self.odom_state.y = y
            self.odom_state.theta = theta
            self.odom_state.vx = vx
            self.odom_state.vy = vy
            self.odom_state.vt = vt

            self.parsed_pub.publish(aiopubsub.Key('odom'), self.odom_state)
            # self.logger.info(f"{x}, {y}, {theta}, {vx}, {vy}, {vt}")
        elif result.category == "ms":
            num_channels = result.get_int(1, signed=False)
            while len(self.module_states.states) != num_channels:
                if len(self.module_states.states) < num_channels:
                    self.module_states.states.append(ModuleState())
                elif len(self.module_states.states) > num_channels:
                    self.module_states.states = [ModuleState() for _ in range(num_channels)]

        elif result.category == "mo":
            channel = result.get_int(1, signed=False)
            if channel < len(self.module_states.states):
                state = self.module_states.states[channel]
                state.timestamp = time.monotonic()
                state.azimuth = result.get_float()
                state.wheel_position = result.get_double()
                state.wheel_velocity = result.get_float()
                if channel + 1 == len(self.module_states.states):
                    self.parsed_pub.publish(aiopubsub.Key('module'), self.module_states)
        
        elif result.category == "power":
            voltage = result.get_float()
            current = result.get_float()
            self.logger.info(f"{voltage: 0.4f} V, {current: 0.4f} A")
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
