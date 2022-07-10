import asyncio
from .util import *


class PacketResult:
    """
    Object representing all properies extracted from a packet including:
    - category
    - data segments
    - packet type

    Usage:
    assuming we're looking for a packet of category "thing" and it contains a float then two integers:
    if result.category == "thing":
        first = result.get_float()
        second = result.get_int()
        third = result.get_int()
    """
    def __init__(self, error_code, recv_time, packet_num):
        self.error_code = error_code
        self.recv_time = recv_time

        self.category = ""  # category extracted from packet
        self.buffer = b''  # bytes containing a partial packet
        self.start_index = 0  # where in the buffer to start looking for data
        self.stop_index = 0  # where in the buffer to stop looking for data
        self.current_index = 0  # where in the buffer we're currently at
        self.packet_type = PACKET_TYPE_NORMAL  # NORMAL, HANDSHAKE, or CONFIRMING
        self.packet_num = packet_num  # read packet number
        self.use_double_precision = True  # whether to use 8-byte or 4-byte precision in get_float. Set by protocol
    
    def set_start_index(self, index):
        """Set data segment start index. Used in protocol. Not for external use"""
        self.start_index = index
        self.current_index = self.start_index
    
    def set_stop_index(self, index):
        """Set data segment stop index. Used in protocol. Not for external use"""
        self.stop_index = index

    def set_buffer(self, buffer):
        """Set buffer. Used in protocol. Not for external use"""
        self.buffer = buffer
    
    def get_packet(self):
        return self.buffer
    
    def set_error_code(self, error_code):
        """Set packet error code. Used in protocol. Not for external use"""
        if error_code == NULL_ERROR:
            return
        self.error_code = error_code
    
    def set_category(self, category):
        """Set packet category. Used in protocol. Not for external use"""
        self.category = category
    
    def set_type(self, packet_type):
        """Set packet type. Used in protocol. Not for external use"""
        self.packet_type = packet_type
    
    def set_packet_num(self, packet_num):
        """Set packet number. Used in protocol. Not for external use"""
        self.packet_num = packet_num

    def check_index(self):
        """Check if buffer has been exceeded while parsing data segments"""
        if self.current_index >= self.stop_index:
            raise RuntimeError("Index exceeds buffer limits. %d >= %d" % (self.current_index, self.stop_index))
    
    def get_int(self) -> int:
        """Parse the next 4 bytes in the packet as an integer. Return int value"""
        next_index = self.current_index + 4
        result = to_int(self.buffer[self.current_index: next_index])
        self.current_index = next_index
        self.check_index()
        return result

    def get_float(self) -> float:
        """
        Parse the next 4 or 8 bytes in the packet as an float (depends on use_double_precision).
        Return float value
        """
        if self.use_double_precision:
            next_index = self.current_index + 8
            result = to_double(self.buffer[self.current_index: next_index])
        else:
            next_index = self.current_index + 4
            result = to_float(self.buffer[self.current_index: next_index])
        self.current_index = next_index
        self.check_index()
        return result

    def get_string(self, length=None) -> str:
        """
        Parse the next X bytes in the packet as a string.
        Length is encoded in first two bytes of string. Return str value
        """
        if length is None:
            next_index = self.current_index + 2
            length = to_int(self.buffer[self.current_index: next_index])

        next_index = self.current_index + length
        result = self.buffer[self.current_index: next_index].decode()
        self.current_index = next_index
        self.check_index()
        return result

    def copy_from(self, other):
        if isinstance(other, self.__class__):
            raise AttributeError("Can't copy from a non-PacketResult object: %s" % repr(other))
        self.error_code = other.error_code
        self.recv_time = other.recv_time
        self.category = other.category
        self.buffer = other.buffer
        self.start_index = other.start_index
        self.stop_index = other.stop_index
        self.current_index = other.current_index
        self.packet_type = other.packet_type
        self.packet_num = other.packet_num
        self.use_double_precision = other.use_double_precision

    def __hash__(self) -> int:
        """Hash function for PacketResult. For comparing PacketResult objects for equality"""
        return hash((self.category, self.packet_num))

    def __eq__(self, other):
        """Check if PacketResult are equivalent using the hash function"""
        if isinstance(other, self.__class__):
            return hash(other) == hash(self)
        else:
            return False

    def __str__(self) -> str:
        return f"{self.__class__.__name__}<category={self.category}, error_code={self.error_code}, recv_time={self.recv_time}, packet_num={self.packet_num}, packet_type={self.packet_type}>"

    __repr__ = __str__


class FuturePacketResult(PacketResult):
    def __init__(self, category):
        super().__init__(NULL_ERROR, 0, 0)
        self.category = category
        self.event = asyncio.Event()

    async def wait(self, timeout=None):
        if timeout is None:
            await self.event.wait()
        else:
            await asyncio.wait_for(self.event.wait(), timeout)

    def set_event(self):
        self.event.set()
