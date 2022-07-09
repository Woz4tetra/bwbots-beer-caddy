import time
import warnings
from .result import PacketResult
from .handshake import Handshake
from .util import *


class TunnelProtocol:
    """
    Defines TunnelProtocol. How packets are parsed and interpreted. Generic to any stream of bytes.

    Example of a packet:
    b'\x12\x13\x00\x17\x00\x00\x00\x01\x05cmd\t\x9a\x99\xa9@ff\x06@33\xd3\xc06a\n'
    \x12\x13
        starting bytes. Every packet starts with these two bytes. Defined in
        util.PACKET_START_0 and util.PACKET_START_1
    \x00\x17
        length bytes. Two bytes encode the length of the packet. All bytes up to but not including util.PACKET_STOP
        are included in the length count.
    \x00
        packet type. One byte that determines if the packet is NORMAL, HANDSHAKE, or CONFIRMING.
        NORMAL packets are sent one way with no expectation of a returning packet.
        HANDSHAKE packets expect a CONFIRMING packet in response within a set time duration, otherwise an error is
            thrown.
        CONFIRMING packets are respond to a Handshake packet. They always contain the packet number of the Handshake
            and error code in the data segments.
    \x00\x00\x01\x05
        packet count. Four bytes encode the packet count of the sender. This example interprets to 261 implying
        260 packets have been sent prior to this one.
    cmd\t
        category. "cmd" is the category. \t indicates the end of the category and is not included in the category
        itself. Categories cannot have \t in their definition.
    \x9a\x99\xa9@
    ff\x06@
    33\xd3\xc0
        data bytes. It's implied the users of this protocol know what order the data is coming in. Here it's
        assumed this data encodes 3, 32-bit floats: 5.3, 2.1, -6.6
        use_double_precision was set to False for this example. If true, floats use 8-bytes
    6a
        checksum bytes. length, packet count, category, and data bytes are summed ignoring overflow.
        The resulting 8-bit integer is encoded as a hexadecimal string. When the receiver parses the packet,
        it will do the same math and should hopefully get the same result encoded in the checksum.
    \n
        stop byte. Every packet ends with this byte. Defined in util.PACKET_STOP
    """

    def __init__(self, max_packet_len=128, debug=False):
        """
        :param max_packet_len: Maximum allowable packet length. Set based on device with the smallest memory
        """
        self.debug = debug
        self.max_packet_len = max_packet_len
        self.min_packet_len = 0  # the minimum packet length. Filled in later by calculation
        self.max_segment_len = max_packet_len  # the maximum packet length. Calculated later

        self.read_packet_num = -1  # internal counter for number of packets received
        self.recv_packet_num = 0  # last received packet number
        self.write_packet_num = 0  # internal counter for number of packets written
        self.dropped_packet_num = 0  # internal counter for number of packets that had an error

        self.buffer_index = 0  # after a packet has been verified, this tracks where the next segment starts
        self.current_segment = b''  # raw data contained within the last segment
        self.use_double_precision = True  # If True, use 8 byte floats. If False, use 4 byte floats

        # mapping of packet codes to printable strings
        self.packet_error_codes = {
            NULL_ERROR: "packet result wasn't properly initialized",
            NO_ERROR: "no error",
            PACKET_0_ERROR: "c1 != %s" % str(PACKET_START_0),
            PACKET_1_ERROR: "c2 != %s" % str(PACKET_START_1),
            PACKET_TOO_SHORT_ERROR: "packet is too short",
            CHECKSUMS_DONT_MATCH_ERROR: "checksums don't match",
            PACKET_COUNT_NOT_FOUND_ERROR: "packet count segment not found",
            PACKET_COUNT_NOT_SYNCED_ERROR: "packet counts not synchronized",
            PACKET_CATEGORY_ERROR: "failed to find category segment",
            INVALID_FORMAT_ERROR: "invalid format",
            PACKET_STOP_ERROR: "packet didn't end with stop character",
            SEGMENT_TOO_LONG_ERROR: "packet segment is too long",
            PACKET_TIMEOUT_ERROR: "packet receive timed out",
        }

        self.minimum_packet = self.make_packet("x")  # create the smallest packet
        self.write_packet_num = 0  # reset write_packet_num again
        self.min_packet_len = len(self.minimum_packet)  # determine minimum packet length from smallest packet

        # find smallest segment length from packet. The maximum is max_packet_len - min_packet_len
        # or the maximum value of 2 bytes since that's maximum length allowed for string parsing in this protocol
        self.max_segment_len = min(self.max_packet_len - self.min_packet_len, 0xffff)

    def make_handshake_packet(self, category: str, *args, write_interval=0.0, timeout=1.0) -> Handshake:
        """
        Create a Handshake packet. The purpose of this packet is to ensure the device got the message
        otherwise throw an error. Optionally, resend the packet at fixed intervals if a confirm packet isn't received
        :param category: str, category of packet. For packet routing on the receiving end. Must not contain:
            util.PACKET_SEP_STR (\t)
        :param args: objects to interpret into a packet. Accepted types: int, str, bytes, float
        :param write_interval: how often to resend the created packet
        :param timeout: how to wait before throwing an error
        :return: Handshake object containing packet and metadata
        """
        packet = self.make_packet(category, *args, packet_type=PACKET_TYPE_HANDSHAKE)
        return Handshake(category, packet, self.write_packet_num - 1, write_interval, timeout)

    def make_confirming_packet(self, category, packet_num) -> bytes:
        """
        Create a confirming packet. This packet is sent in response to receiving a handshake packet type.
        Its contents are the packet number the confirming packet is responding to and the client's error code for
        the handshake packet.
        :param category: str, category of packet. For packet routing on the receiving end. Must not contain:
            util.PACKET_SEP_STR (\t)
        :param packet_num: Packet number of the handshake packet
        :return: bytes
        """
        return self.make_packet(category, packet_num, packet_type=PACKET_TYPE_CONFIRMING)

    def make_packet(self, category, *args, packet_type=PACKET_TYPE_NORMAL) -> bytes:
        """
        Create a normal packet using the provided arguments
        :param category: str, category of packet. For packet routing on the receiving end. Must not contain:
            util.PACKET_SEP_STR (\t)
        :param args: objects to interpret into a packet. Accepted types: int, str, bytes, float
        :param packet_type: util.PACKET_TYPE_NORMAL, util.PACKET_TYPE_HANDSHAKE, or util.PACKET_TYPE_CONFIRMING
        :return: bytes
        """
        packet = self.packet_header(category, packet_type)  # start packet with the protocol header
        for arg in args:
            # Parse each argument into bytes and append them to the packet
            if type(arg) == int:
                packet += to_int32_bytes(arg)
            elif type(arg) == float:
                if self.use_double_precision:
                    packet += to_double_bytes(arg)
                else:
                    packet += to_float_bytes(arg)
            elif type(arg) == str or type(arg) == bytes:
                # for strings and bytes, first determine the length of the string.
                # insert that value as an integer represented by two bytes,
                # then insert the data bytes
                if len(arg) >= self.max_segment_len:
                    raise TunnelProtocolException("Segment exceeds maximum segment length: %s" % repr(arg))
                len_bytes = to_uint16_bytes(len(arg))
                if type(arg) == str:
                    arg = arg.encode()
                packet += len_bytes + arg
            else:
                warnings.warn("Invalid argument type: %s, %s" % (type(arg), arg))

        packet = self.packet_footer(packet)  # append footer as well as insert length bytes into the beginning
        if len(packet) > self.max_packet_len:
            raise TunnelProtocolException("Packet exceeds maximum allowable length: %s" % repr(packet))
        if len(packet) < self.min_packet_len:
            raise TunnelProtocolException("Packet exceeds minimum allowable length: %s" % repr(packet))

        self.write_packet_num += 1
        return packet

    def packet_header(self, category: str, packet_type: int) -> bytes:
        """
        Create segments that are included in the length count that aren't the data and the checksum:
        packet_type, write_packet_num, and category + util.PACKET_SEP
        """
        category = str(category).encode()
        if PACKET_SEP in category:
            raise TunnelProtocolException("Cannot have %s in the category: %s" % (PACKET_SEP, repr(category)))
        packet = to_uint8_bytes(packet_type)
        packet += to_int32_bytes(self.write_packet_num)
        packet += category + PACKET_SEP
        return packet

    def packet_footer(self, packet: bytes) -> bytes:
        """
        Insert starting bytes (util.PACKET_START_0 + util.PACKET_START_1), calculate and insert length bytes,
        calculate and append checksum bytes, and delimiting byte (util.PACKET_STOP).
        :param packet: bytes, packet with length counting bytes
        :return: bytes, completed packet
        """
        calc_checksum = self.calculate_checksum(packet)

        packet += b"%02x" % calc_checksum  # convert checksum int to hexadecimal string

        packet_len = len(packet)  # calculate length starting from length bytes up to the end of the checksum
        packet_len_bytes = to_uint16_bytes(packet_len)  # encode length as 2 bytes

        packet = PACKET_START_0 + PACKET_START_1 + packet_len_bytes + packet  # insert starting and length bytes
        packet += PACKET_STOP  # append stop byte

        return packet

    @staticmethod
    def calculate_checksum(packet: bytes) -> int:
        """
        Calculate checksum from provided bytes. Should include length, packet count, category, and data bytes only
        :param packet: bytes, Packet to calculate checksum from
        :return: int, encoding the checksum
        """
        calc_checksum = 0
        for val in packet:
            calc_checksum += val
        calc_checksum &= 0xff

        return calc_checksum

    @staticmethod
    def extract_checksum(packet: bytes) -> int:
        """
        Take the last two bytes of a packet and interpret them as a hexadecimal string
        :param packet: bytes of the received packet. At least length two
        :return: int, extracted checksum
        """
        try:
            return int(packet[-2:], 16)
        except ValueError as e:
            warnings.warn("Failed to parse checksum as int: %s" % str(e))
            return -1

    def parse_buffer(self, buffer: bytes) -> tuple:
        """
        Parse buffer of characters into as many packet results as possible.
        Returns characters not used in parsing
        :param buffer: bytes to parse
        :return: Tuple[bytes, bytes, List[PacketResult]]
            all bytes not used in packet parsing,
            trailing bytes in the buffer not used for packet parsing,
            list of PacketResult's (empty if no packets parsed)
        """
        index = -1  # index is incremented immediately, start from -1 to capture index #0
        results = []  # store packet results here
        last_packet_index = 0  # index of last PACKET_STOP encountered that had start bytes and correct length
        remaining_buffer = b""  # store debug statement bytes here

        # iterate over the entire buffer one character at a time
        while index < len(buffer):
            index += 1
            packet_start = index  # if the checks below pass, packet_start will remain at the beginning of the packet

            # assume data that doesn't start with util.PACKET_START_0 is part of a debug message
            if buffer[index:index + 1] != PACKET_START_0:
                remaining_buffer += buffer[index:index + 1]
                continue
            index += PACKET_START_LENGTH  # move to the next index if util.PACKET_START_0 is encountered
            if index >= len(buffer):
                # exit if the buffer is overrun, the packet is incomplete
                break

            # assume data that doesn't start with util.PACKET_START_1 is part of a debug message
            if buffer[index:index + 1] != PACKET_START_1:
                remaining_buffer += buffer[index:index + 1]
                continue
            index += PACKET_START_LENGTH  # move to the next index if util.PACKET_START_1 is encountered
            if index >= len(buffer):
                # exit if the buffer is overrun, the packet is incomplete
                break

            raw_length = buffer[index:index + 2]  # per definition, next two bytes must be length
            # length must be between 0 and the length of the remaining buffer.
            if len(raw_length) == 0:
                # length is invalid. Start searching for packet start
                continue
            if index + PACKET_LEN_LENGTH >= len(buffer):
                # exit if the buffer is overrun, the packet is incomplete
                break
            index += PACKET_LEN_LENGTH  # move index to after the length bytes. This is where length is calculated from

            length = to_int(raw_length)  # interpret two length bytes as an integer
            if index + length >= len(buffer):
                # exit if the buffer is overrun, the packet is incomplete
                break
            if buffer[index + length: index + length + 1] != PACKET_STOP:
                # packet doesn't end with util.PACKET_STOP, it must be corrupted. Search for a new packet
                # from the next index onward
                continue
            index += length  # move the specified number of bytes along the buffer

            # from this point don't modify index otherwise parts of the next packet will be skipped
            # since the index immediately gets incremented at the beginning of the loop
            last_packet_index = index + 1  # include util.PACKET_STOP in last_packet_index
            packet = buffer[packet_start:last_packet_index]  # slice buffer according to start and stop
            if self.debug:
                print("Received packet:", packet)
            result = self.parse_packet(packet)  # parse found packet into PacketResult
            if result.error_code != NO_ERROR:
                self.dropped_packet_num += 1  # any error code that isn't util.NO_ERROR counts as a dropped packet
            self.read_packet_num += 1
            results.append(result)

        return remaining_buffer, buffer[last_packet_index:], results

    def parse_packet(self, packet: bytes) -> PacketResult:
        """
        Parse received bytes identified as a potential valid packet into PacketResult
        :param packet: bytes to parse
        :return: PacketResult
        """
        recv_time = time.time()
        if len(packet) < self.min_packet_len:  # check minimum length constraint
            warnings.warn("Packet is not the minimum length (%s): %s" % (self.min_packet_len, repr(packet)))
            return PacketResult(PACKET_TOO_SHORT_ERROR, recv_time, self.read_packet_num)

        if packet[0:1] != PACKET_START_0:  # verify packet actually starts with util.PACKET_START_0
            warnings.warn("Packet does not start with PACKET_START_0: %s" % repr(packet))
            return PacketResult(PACKET_0_ERROR, recv_time, self.read_packet_num)
        if packet[1:2] != PACKET_START_1:  # verify packet actually starts with util.PACKET_START_1
            warnings.warn("Packet does not start with PACKET_START_1: %s" % repr(packet))
            return PacketResult(PACKET_1_ERROR, recv_time, self.read_packet_num)
        if packet[-1:] != PACKET_STOP:  # verify packet actually ends with util.PACKET_STOP
            warnings.warn("Packet does not stop with PACKET_STOP: %s" % repr(packet))
            return PacketResult(PACKET_STOP_ERROR, recv_time, self.read_packet_num)

        full_packet = packet  # sequester packet for debug printing
        packet = packet[PACKET_HEADER_LENGTH:-1]  # remove start bytes, length, and stop bytes
        calc_checksum = self.calculate_checksum(packet[:-PACKET_CHECKSUM_LENGTH])  # calculate checksum from packet
        recv_checksum = self.extract_checksum(packet)  # extract checksum from packet
        if recv_checksum != calc_checksum:  # if they don't match, packet is corrupted
            warnings.warn(
                "Checksum failed! recv %02x != calc %02x. %s" % (recv_checksum, calc_checksum, repr(full_packet)))
            return PacketResult(CHECKSUMS_DONT_MATCH_ERROR, recv_time, self.read_packet_num)

        packet = packet[:-PACKET_CHECKSUM_LENGTH]  # remove checksum

        # everything in the buffer has been removed except for packet type, count, category, and data.
        # Reset segment finder index
        self.buffer_index = 0

        # extract packet type byte and put it in self.current_segment
        if not self.get_next_segment(packet, PACKET_TYPE_LENGTH):
            warnings.warn("Failed to find packet number segment! %s" % (repr(full_packet)))
            return PacketResult(PACKET_TYPE_NOT_FOUND_ERROR, recv_time, self.read_packet_num)
        packet_type = to_int(self.current_segment)  # parse packet type byte as an integer
        if packet_type not in PACKET_TYPES:  # if not a valid packet type, signal a packet error
            warnings.warn("Failed to find valid packet type! %s. Found: %s" % (repr(full_packet), packet_type))
            return PacketResult(PACKET_TYPE_NOT_FOUND_ERROR, recv_time, self.read_packet_num)

        # extract packet num bytes and put them in self.current_segment
        if not self.get_next_segment(packet, PACKET_COUNT_LENGTH):
            warnings.warn("Failed to find packet number segment! %s" % (repr(full_packet)))
            return PacketResult(PACKET_COUNT_NOT_FOUND_ERROR, recv_time, self.read_packet_num)
        self.recv_packet_num = to_int(self.current_segment)  # parse four type bytes as an integer

        # instantiate packet result we might return. If a non critical error occurs, we'll still return
        # a PacketResult with everything we found just with a "warning" error code
        packet_result = PacketResult(NO_ERROR, recv_time, self.read_packet_num)
        packet_result.use_double_precision = self.use_double_precision

        # if this is the first packet received, reset count to match the just received packet
        if self.read_packet_num == -1:
            self.read_packet_num = self.recv_packet_num

        # signal a warning if packet count doesn't match. This potentially signals a dropped packet
        if self.recv_packet_num != self.read_packet_num:
            warnings.warn("Received packet num doesn't match local count. "
                          "recv %s != local %s" % (self.recv_packet_num, self.read_packet_num))
            # print("Buffer: %s" % packet)
            self.read_packet_num = self.recv_packet_num
            packet_result.set_error_code(PACKET_COUNT_NOT_SYNCED_ERROR)

        # extract category bytes and put them in self.current_segment, delimited by util.PACKET_SEP
        if not self.get_next_segment(packet, tab_separated=True):
            warnings.warn(
                "Failed to find category segment %s! %s" % (repr(self.current_segment), repr(full_packet)))
            return PacketResult(PACKET_CATEGORY_ERROR, recv_time, self.read_packet_num)
        try:
            # attempt to decode bytes to a string. If it fails, the category is invalid
            category = self.current_segment.decode()
        except UnicodeDecodeError:
            warnings.warn("Category segment contains invalid characters: %s, %s" % (
                repr(self.current_segment), repr(full_packet)))
            return PacketResult(PACKET_CATEGORY_ERROR, recv_time, self.read_packet_num)

        if len(category) == 0:  # category must have at least length 1
            warnings.warn("Category segment is empty: %s, %s" % (
                repr(self.current_segment), repr(full_packet)))
            return PacketResult(PACKET_CATEGORY_ERROR, recv_time, self.read_packet_num)

        if len(category) > self.max_segment_len:  # category must not exceed segment length
            warnings.warn("Category segment is too large: %s, %s" % (
                repr(self.current_segment), repr(full_packet)))
            return PacketResult(PACKET_CATEGORY_ERROR, recv_time, self.read_packet_num)

        # self.buffer_index is now at where the data starts
        # Tell the PacketResult where the data starts and ends
        packet_result.set_start_index(self.buffer_index)
        packet_result.set_stop_index(len(packet) + 1)
        packet_result.set_buffer(packet)

        # set category and packet type metadata
        packet_result.set_category(category)
        packet_result.set_type(packet_type)

        return packet_result

    def get_next_segment(self, buffer: bytes, length=None, tab_separated=False) -> bool:
        """
        Put data into self.current_segment according to two schema:
        - length: increment self.buffer_index by this much after
        - tab_separated: find next instance util.PACKET_SEP after self.buffer_index. Increment self.buffer_index
            by that much.
        :param buffer: bytes to search in
        :param length: length to advance. Ignored if tab_separated is True.
        :param tab_separated: whether to advance by finding the next util.PACKET_SEP
        :return: False if the buffer was exceeded by advancing. If tab_separated is True, the buffer was exceeded
            while searching for util.PACKET_SEP
        """
        if self.buffer_index >= len(buffer):
            return False
        if tab_separated:
            sep_index = buffer.find(PACKET_SEP, self.buffer_index)
            if sep_index == -1:
                self.current_segment = buffer[self.buffer_index:]
                self.buffer_index = len(buffer)
            else:
                self.current_segment = buffer[self.buffer_index: sep_index]
                self.buffer_index = sep_index + 1
            return True
        else:
            if length is None:
                raise TunnelProtocolException(
                    "length must not be None or tab_separated must be True in order to parse a segment"
                )
            self.current_segment = buffer[self.buffer_index: self.buffer_index + length]
            self.buffer_index += length
            return True

    def is_code_error(self, error_code: int) -> bool:
        """
        Check list of error codes that don't warrant an error routine. If error_code is not in it,
        return True

        :param error_code: int
        :return: whether error_code is an error not a warning
        """
        return error_code not in PACKET_WARNINGS

    def is_handshake_confirm(self, result: PacketResult) -> bool:
        """
        Check if PacketResult contains a confirming packet type
        :param result: PacketResult returned from parse
        :return: whether result is a confirming packet or not
        """
        return result.packet_type == PACKET_TYPE_CONFIRMING

    def log_packet_error_code(self, error_code: int, packet_num=None):
        """Print a warning if error code is an error. Interprets error_code int into a readable string"""
        if packet_num is None:
            packet_num = self.read_packet_num
        if error_code == NO_ERROR:
            # print("Packet %s has no error" % packet_num)
            return

        if not self.is_code_error(error_code):
            message = "Packet %s returned a warning:" % packet_num
        else:
            message = "Packet %s returned an error:" % packet_num
        if error_code in self.packet_error_codes:
            message += "\t%s" % self.packet_error_codes[error_code]
        else:
            message += "\tUnknown error code: %s" % error_code
        warnings.warn(message)
