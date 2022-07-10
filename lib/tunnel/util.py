import struct

# Packet parse error code
NULL_ERROR = -1
NO_ERROR = 0
PACKET_0_ERROR = 1
PACKET_1_ERROR = 2
PACKET_TOO_SHORT_ERROR = 3
CHECKSUMS_DONT_MATCH_ERROR = 4
PACKET_COUNT_NOT_FOUND_ERROR = 5
PACKET_COUNT_NOT_SYNCED_ERROR = 6
PACKET_CATEGORY_ERROR = 7
INVALID_FORMAT_ERROR = 8
PACKET_STOP_ERROR = 9
SEGMENT_TOO_LONG_ERROR = 10
PACKET_TIMEOUT_ERROR = 11
PACKET_TYPE_NOT_FOUND_ERROR = 12

PACKET_WARNINGS = [
    NO_ERROR,
    PACKET_COUNT_NOT_SYNCED_ERROR,
]

# Protocol special characters
PACKET_START_0 = b'\x12'
PACKET_START_1 = b'\x13'
PACKET_STOP = b'\n'
PACKET_SEP = b'\t'
PACKET_SEP_STR = '\t'

# Protocol fixed lengths
PACKET_START_LENGTH = 1
PACKET_LEN_LENGTH = 2
PACKET_TYPE_LENGTH = 1
PACKET_COUNT_LENGTH = 4
PACKET_CHECKSUM_LENGTH = 2

PACKET_HEADER_LENGTH = PACKET_START_LENGTH + PACKET_START_LENGTH + PACKET_LEN_LENGTH

# Packet types
PACKET_TYPE_NORMAL = 0
PACKET_TYPE_HANDSHAKE = 1
PACKET_TYPE_CONFIRMING = 2

PACKET_TYPES = (PACKET_TYPE_NORMAL, PACKET_TYPE_HANDSHAKE, PACKET_TYPE_CONFIRMING)


# Exception classes


class HandshakeFailedException(BaseException):
    """
    Handshake didn't return success within the specified amount of time
    """


class TunnelProtocolException(BaseException):
    """
    A part of the protocol was invalidated by user input
    """


# byte conversion helper methods


def to_uint16_bytes(integer):
    return integer.to_bytes(2, 'big')


def to_uint8_bytes(integer):
    return integer.to_bytes(1, 'big')


def to_int_bytes(integer, length=4, signed=True):
    return integer.to_bytes(length, 'big', signed=signed)


def to_double_bytes(floating_point):
    return struct.pack('d', floating_point)


def to_float_bytes(floating_point):
    return struct.pack('f', floating_point)


def to_int(raw_bytes, signed=True):
    return int.from_bytes(raw_bytes, 'big', signed=signed)


def to_double(raw_bytes):
    return struct.unpack('d', raw_bytes)[0]


def to_float(raw_bytes):
    return struct.unpack('f', raw_bytes)[0]
