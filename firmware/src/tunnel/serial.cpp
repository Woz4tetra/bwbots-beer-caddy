
#include "tunnel/serial.h"

char* _read_buffer = new char[TunnelProtocol::MAX_PACKET_LEN];
char* _write_buffer = new char[TunnelProtocol::MAX_PACKET_LEN];
uint32_t start_wait_time;
bool _initialized = false;
TunnelProtocol* _protocol;
PacketResult* _result;

void tunnel_begin()
{
    PROTOCOL_SERIAL.begin(PROTOCOL_BAUD);
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(DEBUG_BAUD);
#endif
    _initialized = true;
    _protocol = new TunnelProtocol();
    _result = new PacketResult(TunnelProtocol::NULL_ERROR, 0);

    for (int index = 0; index < TunnelProtocol::MAX_PACKET_LEN; index++) {
        _read_buffer[index] = '\0';
    }
}

PacketResult* tunnel_readPacket()
{
    if (!PROTOCOL_SERIAL.available()) {
        return NULL;
    }

    char c = PROTOCOL_SERIAL.read();
    if (c == TunnelProtocol::PACKET_START_0) {
       start_wait_time = millis();
        while (!PROTOCOL_SERIAL.available()) {
            if (millis() - start_wait_time > PACKET_STOP_TIMEOUT) {
                DEBUG_SERIAL.println(F("Time out exceeded for start"));
                return NULL;
            }
        }
        c = PROTOCOL_SERIAL.read();
        if (c != TunnelProtocol::PACKET_START_1) {
            return NULL;
        }
    }
    else {
        return NULL;
    }
    int _num_chars_read = 0;
    _read_buffer[_num_chars_read++] = TunnelProtocol::PACKET_START_0;
    _read_buffer[_num_chars_read++] = TunnelProtocol::PACKET_START_1;
    
    start_wait_time = millis();
    int packet_len = 0;
    while (true)
    {
        if (millis() - start_wait_time > PACKET_STOP_TIMEOUT) {
            DEBUG_SERIAL.println(F("Time out exceeded"));
            break;
        }
        if (!PROTOCOL_SERIAL.available()) {
            continue;
        }

        c = PROTOCOL_SERIAL.read();
        _read_buffer[_num_chars_read++] = c;
        if (_num_chars_read >= TunnelProtocol::MAX_PACKET_LEN) {
            DEBUG_SERIAL.println(F("Max num chars exceeded"));
            return NULL;
        }
        if (_num_chars_read == TunnelProtocol::CHECKSUM_START_INDEX) {
            packet_len = (int)to_uint16(_read_buffer + TunnelProtocol::LENGTH_START_INDEX);
        }
        else if (_num_chars_read > TunnelProtocol::CHECKSUM_START_INDEX) {
            if (packet_len >= TunnelProtocol::MAX_PACKET_LEN) {
                DEBUG_SERIAL.println(F("Max packet len exceeded"));
                return NULL;
            }
            if (_num_chars_read - TunnelProtocol::CHECKSUM_START_INDEX > packet_len)
            {
                if (c != TunnelProtocol::PACKET_STOP) {
                    DEBUG_SERIAL.print(F("_num_chars_read: "));
                    DEBUG_SERIAL.println(_num_chars_read);
                    DEBUG_SERIAL.println(F("Last char not stop"));
                    return NULL;
                }
                break;
            }
        }
    }
    _read_buffer[_num_chars_read] = '\0';
    
    // DEBUG_SERIAL.print(F("_num_chars_read: "));
    // DEBUG_SERIAL.println(_num_chars_read);

    // DEBUG_SERIAL.print(F("packet_len: "));
    // DEBUG_SERIAL.println(packet_len);

    // for (int index = 0; index < TunnelProtocol::MAX_PACKET_LEN; index++) {
    //     DEBUG_SERIAL.print(_read_buffer[index], HEX);
    //     DEBUG_SERIAL.print(' ');
    // }
    // DEBUG_SERIAL.print('\n');

    _result->setErrorCode(TunnelProtocol::NULL_ERROR);
    _protocol->parsePacket(_read_buffer, 0, _num_chars_read, _result);
    int code = _result->getErrorCode();
    if (code == TunnelProtocol::NULL_ERROR) {
        return NULL;
    }
    if (_protocol->isCodeError(code)) {
        DEBUG_SERIAL.print(F("Encountered error code: "));
        DEBUG_SERIAL.println(code);
        return NULL;
    }
    if (_result->getPacketType() == PACKET_TYPE_HANDSHAKE) {
        tunnel_writeConfirmingPacket(_result->getCategory().c_str(), "ud", _result->getPacketNum(), _result->getErrorCode());
        return NULL;
    }
    return _result;
}

// TODO: add tunnel_writeHandshakePacket

void tunnel_writeConfirmingPacket(const char *category, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    int length = _protocol->makePacket(PACKET_TYPE_CONFIRMING, _write_buffer, category, formats, args);
    tunnel_writeBuffer(length);
    va_end(args);
}


void tunnel_writePacket(const char *category, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    int length = _protocol->makePacket(PACKET_TYPE_NORMAL, _write_buffer, category, formats, args);
    tunnel_writeBuffer(length);
    va_end(args);
}

void tunnel_writeBuffer(int length)
{
    if (!_initialized) {
        DEBUG_SERIAL.println(F("Device is not initialized. Skipping write"));
        return;
    }
    // REPORT_ERROR("Writing packet", packetToString(_write_buffer, 0, length).c_str());
    if (0 < length && length < TunnelProtocol::MAX_PACKET_LEN) {
        PROTOCOL_SERIAL.write(_write_buffer, length);
    }
    else {
        DEBUG_SERIAL.println(F("Skipping write for packet"));
    }
}
