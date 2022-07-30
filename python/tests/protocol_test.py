import sys
sys.path.insert(0, "..")

from lib.tunnel.protocol import TunnelProtocol
from lib.tunnel.util import *

if __name__ == '__main__':
    def main():
        protocol = TunnelProtocol()
        protocol.use_double_precision = True

        data = "ping", "f", 4.0
        test_packet = protocol.make_packet(*data)
        print(test_packet)
        _, _, results = protocol.parse_buffer(test_packet)
        result = results[0]
        parsed = (result.category, "f", result.get_float())
        protocol.log_packet_error_code(result.error_code)
        assert data == parsed, "%s != %s" % (data, parsed)
        print(result)

        protocol.write_packet_num = 260
        protocol.read_packet_num = 260
        # data = "something", 50.0, 10, b"else"
        data = "cmd", "fff", 5.3, 2.1, -6.6
        test_packet = protocol.make_packet(*data)
        print(test_packet)
        _, _, results = protocol.parse_buffer(test_packet)
        result = results[0]
        parsed = (result.category, "fff", result.get_float(), result.get_float(), result.get_float())
        protocol.log_packet_error_code(result.error_code)
        # assert data == parsed, "%s != %s" % (data, parsed)
        print(result)

        buffer = b'\x12\x13\x00\x17\x00\x00\x00\x01\x05cmd\t\x9a\x99\xa9@ff\x06@33\xd3\xc06a\n'
        _, _, results = protocol.parse_buffer(buffer)

        result = results[0]
        assert result.error_code == NO_ERROR, result.error_code
        assert result.category == "cmd", repr(result.category)
        assert result.packet_type == PACKET_TYPE_NORMAL
        assert result.packet_num == 261
        assert abs(result.get_float() - 5.3) < 0.001
        assert abs(result.get_float() - 2.1) < 0.001
        assert abs(result.get_float() + 6.6) < 0.001


        buffer = b'\x12\x13\x00$\x01\x00\x00\x00\x01balance\t5c\xd1tv2\xb0?\x00\x034.0\x9a\x99\x99\x99\x99\x99\xa9?59\n'
        _, _, results = protocol.parse_buffer(buffer)
        result = results[0]
        print(result.get_double())
        print(result.get_double())
        # print(result.get_double())

    main()
