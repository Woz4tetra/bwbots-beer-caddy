#pragma once

#include <limits.h>
#include <Arduino.h>


#define DEBUG_SERIAL Serial
#define DEBUG_BAUD 57600


void REPORT_ERROR(const char* report, ...);

typedef union uint16_union
{
    uint16_t integer;
    unsigned char byte[sizeof(uint16_t)];
} uint16_union_t;

typedef union uint32_union
{
    uint32_t integer;
    unsigned char byte[sizeof(uint32_t)];
} uint32_union_t;


typedef union int32_union
{
    int32_t integer;
    unsigned char byte[sizeof(int32_t)];
} int32_union_t;

typedef union float_union
{
    float floating_point;
    unsigned char byte[sizeof(float)];
} float_union_t;

typedef union double_union
{
    double floating_point;
    unsigned char byte[sizeof(double)];
} double_union_t;


uint32_t to_uint32(char* buffer);
uint16_t to_uint16(char* buffer);
int32_t to_int32(char* buffer);
float to_float(char* buffer);
double to_double(char* buffer);
String to_string(char* buffer, int length);
uint8_t from_checksum(char* buffer);
String format_char(unsigned char c);
String packetToString(char* buffer, int start_index, int stop_index);

typedef enum packet_type  {
    PACKET_TYPE_NORMAL = 0,
    PACKET_TYPE_HANDSHAKE = 1,
    PACKET_TYPE_CONFIRMING = 2
} packet_type_t;

class PacketResult
{
private:
    String _category;
    int _error_code;
    uint32_t _recv_time;
    int _start_index;
    int _stop_index;
    char* _buffer;
    int _current_index;
    packet_type_t _packet_type;
    uint32_t _packet_num;
    
    bool checkIndex() {
        if (_current_index >= _stop_index) {
#ifdef DEBUG_SERIAL
            REPORT_ERROR("Index exceeds buffer limits. %d >= %d", _current_index, _stop_index);
#else
            DEBUG_SERIAL.println(F("Index exceeds buffer limits"));
#endif
            return false;
        }
        return true;
    }
public:
    PacketResult(int error_code, uint32_t recv_time) {
        _category = "";
        _recv_time = recv_time;
        _error_code = error_code;
        _packet_type = PACKET_TYPE_NORMAL;
        _packet_num = 0;
    }

    ~PacketResult() {
        
    }

    void setCategory(String category) {
        _category = category;
    }
    String getCategory() {
        return _category;
    }
    void setPacketType(packet_type_t packet_type) {
        _packet_type = packet_type;
    }
    packet_type_t getPacketType() {
        return _packet_type;
    }
    void setPacketNum(uint32_t packet_num) {
        _packet_num = packet_num;
    }
    uint32_t getPacketNum() {
        return _packet_num;
    }
    void setErrorCode(int error_code) {
        _error_code = error_code;
    }
    int getErrorCode() {
        return _error_code;
    }
    void setRecvTime(uint32_t recv_time) {
        _recv_time = recv_time;
    }
    uint32_t getRecvTime() {
        return _recv_time;
    }
    void setBuffer(char* buffer) {
        _buffer = buffer;
    }
    char* getBuffer() {
        return _buffer;
    }
    void setStart(int index) {
        _start_index = index;
        _current_index = _start_index;
    }
    int getStart() {
        return _start_index;
    }
    void setStop(int index) {
        _stop_index = index;
    }
    int getStop() {
        return _stop_index;
    }
    bool getInt(int32_t& result) {
        result = to_int32(_buffer + _current_index);
        _current_index += sizeof(int32_t);
        return checkIndex();
    }
    bool getFloat(float& result) {
        result = to_float(_buffer + _current_index);
        _current_index += sizeof(float);
        return checkIndex();
    }
    bool getDouble(double& result) {
        result = to_double(_buffer + _current_index);
        _current_index += sizeof(double);
        return checkIndex();
    }
    bool getString(String& result) {
        int length = to_uint16(_buffer + _current_index);
        _current_index += sizeof(uint16_t);
        getString(result, length);
        return checkIndex();
    }
    bool getString(String& result, int length) {
        result = to_string(_buffer + _current_index, length);
        _current_index += length;
        return checkIndex();
    }
};


#define RESULT_QUEUE_MAX_LEN 10


class TunnelProtocol
{
private:
    int _read_buffer_index;
    uint32_t _read_packet_num, _write_packet_num;
    int _current_segment_start, _current_segment_stop;

    bool getNextSegment(char* buffer, int stop_index);
    bool getNextSegment(char* buffer, int stop_index, int length);

public:
    static const char PACKET_START_0 = 0x12;
    static const char PACKET_START_1 = 0x13;
    static const char PACKET_STOP = '\n';
    static const char PACKET_SEP = '\t';
    static const int MAX_PACKET_LEN = 128;
    static const int MIN_PACKET_LEN = 13;
    static const int MAX_SEGMENT_LEN = MAX_PACKET_LEN - MIN_PACKET_LEN;
    static const int CHECKSUM_START_INDEX = 4;
    static const int LENGTH_START_INDEX = 2;
    static const int LENGTH_BYTE_LENGTH = 2;

    static const int NULL_ERROR = -1;
    static const int NO_ERROR = 0;
    static const int PACKET_0_ERROR = 1;
    static const int PACKET_1_ERROR = 2;
    static const int PACKET_TOO_SHORT_ERROR = 3;
    static const int CHECKSUMS_DONT_MATCH_ERROR = 4;
    static const int PACKET_COUNT_NOT_FOUND_ERROR = 5;
    static const int PACKET_COUNT_NOT_SYNCED_ERROR = 6;
    static const int PACKET_CATEGORY_ERROR = 7;
    static const int INVALID_FORMAT_ERROR = 8;
    static const int PACKET_STOP_ERROR = 9;
    static const int SEGMENT_TOO_LONG_ERROR = 10;
    static const int PACKET_TIMEOUT_ERROR = 11;
    static const int PACKET_TYPE_NOT_FOUND_ERROR = 12;

    TunnelProtocol();
    ~TunnelProtocol();

    void parsePacket(char* buffer, int start_index, int stop_index, PacketResult* result);
    bool isCodeError(int error_code);
    int makePacket(packet_type_t packet_type, char* write_buffer, const char *category, const char *formats, va_list args);
};
