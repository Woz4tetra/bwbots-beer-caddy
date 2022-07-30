import time
from .util import *
from .result import PacketResult


class Handshake:
    """
    Object to track handshake transactions. Contains packet bytes initially sent in case it needs to be resent.
    If specified, handshake packets will be resent at a fixed interval.
    """
    def __init__(self, category: str, packet: bytes, packet_num: int, write_interval: float, timeout: float):
        """
        :param category: category of handshake packet
        :param packet: bytes that were sent the first write
        :param packet_num: write packet num of this packet
        :param write_interval: how often to write the packet if no confirming packet is received.
            If less than or equal to 0.0, the packet will not be written again
        :param timeout: if no confirming packet is received after this time, signal that the handshake failed
        """
        self.category = category
        self.packet = packet
        self.packet_num = packet_num
        self.write_interval = write_interval
        self.timeout = timeout
        self.initial_write_time = time.time()

        self.prev_write_time = self.initial_write_time
        self.attempt_counter = 0
        self.error_code = NULL_ERROR

    @classmethod
    def from_result(cls, packet_result: PacketResult):
        """
        Create a Handshake object from PacketResult.
        The packet result must have two integers in the data segments.
        The first indicates packet number of the initial Handshake.
        The second indicates the error code the sender encountered while processing the handshake.
        """
        packet_num = packet_result.get_int()
        error_code = packet_result.get_int()
        self = cls(packet_result.category, packet_result.get_packet(), packet_num, 0.0, 0.0)
        self.error_code = error_code
        return self

    def should_write_again(self):
        if self.write_interval <= 0.0:  # never write the packet again if interval is less than or equal to zero
            return False
        current_time = time.time()
        dt = current_time - self.prev_write_time
        if dt > self.write_interval:  # reset prev_write_time timer if threshold is exceeded and return True
            self.prev_write_time = current_time
            self.attempt_counter += 1
            return True
        else:
            return False

    def did_fail(self):
        if self.timeout > 0.0:
            return time.time() - self.initial_write_time > self.timeout
        else:
            return False
    
    def __hash__(self) -> int:
        """Hash function for Handshake. For comparing Handshake objects for equality"""
        return hash((self.category, self.packet_num))

    def __eq__(self, other):
        """Check if Handshakes are equivalent using the hash function"""
        if isinstance(other, self.__class__):
            return hash(other) == hash(self)
        else:
            return False

    def __str__(self) -> str:
        return f"{self.__class__.__name__}('{self.category}', {self.packet}, {self.packet_num}, {self.write_interval}, {self.timeout})"

    __repr__ = __str__
