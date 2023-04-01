#pragma once

#include <limits.h>
#include "ros/ros.h"

using namespace std;

#define THROW_EXCEPTION(msg)  throw std::runtime_error(msg)


typedef union uint64_union
{
    uint64_t integer;
    unsigned char byte[sizeof(uint64_t)];
} uint64_union_t;


typedef union int64_union
{
    int64_t integer;
    unsigned char byte[sizeof(int64_t)];
} int64_union_t;

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

typedef union uint16_union
{
    uint16_t integer;
    unsigned char byte[sizeof(uint16_t)];
} uint16_union_t;

typedef union int16_union
{
    int16_t integer;
    unsigned char byte[sizeof(int16_t)];
} int16_union_t;

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



uint64_t to_uint64(char* buffer);
int64_t to_int64(char* buffer);
uint32_t to_uint32(char* buffer);
int32_t to_int32(char* buffer);
uint16_t to_uint16(char* buffer);
int16_t to_int16(char* buffer);
float to_float(char* buffer);
double to_double(char* buffer);
string to_string(char* buffer, int length);
uint8_t from_checksum(char* buffer);
string format_char(unsigned char c);
string packetToString(char* buffer, int start_index, int stop_index);

typedef enum packet_type  {
    PACKET_TYPE_NORMAL = 0,
    PACKET_TYPE_HANDSHAKE = 1,
    PACKET_TYPE_CONFIRMING = 2
} packet_type_t;

class PacketResult
{
private:
    string _category;
    int _error_code;
    ros::Time _recv_time;
    int _start_index;
    int _stop_index;
    char* _buffer;
    int _current_index;
    packet_type_t _packet_type;
    uint32_t _packet_num;
    
    bool checkIndex();
public:
    PacketResult(int error_code, ros::Time recv_time);
    PacketResult(PacketResult* result);
    ~PacketResult();

    void setFrom(PacketResult* result);
    void setCategory(string category);
    string getCategory();
    void setPacketType(packet_type_t packet_type);
    packet_type_t getPacketType();
    void setPacketNum(uint32_t packet_num);
    uint32_t getPacketNum();
    void setErrorCode(int error_code);
    int getErrorCode();
    void setRecvTime(ros::Time recv_time);
    ros::Time getRecvTime();
    void setBuffer(char* buffer);
    char* getBuffer();
    void setStart(int index);
    int getStart();
    void setStop(int index);
    int getStop();
    bool getInt64(int64_t& result);
    bool getUInt64(uint64_t& result);
    bool getInt32(int32_t& result);
    bool getUInt32(uint32_t& result);
    bool getInt16(int16_t& result);
    bool getUInt16(uint16_t& result);
    bool getInt8(int8_t& result);
    bool getUInt8(uint8_t& result);
    bool getBool(bool& result);
    bool getFloat(float& result);
    bool getDouble(double& result);
    bool getString(string& result);
    bool getString(string& result, int length);
};

class Handshake
{
private:
    char* _packet;
    string _category;
    unsigned int _length;
    double _write_interval;
    double _timeout;
    int _error_code;
    uint32_t _packet_num;

    ros::Time _prev_write_time;
    ros::Time _initial_write_time;

public:
    Handshake(string category, char* buffer, unsigned int length, uint32_t packet_num, double write_interval, double timeout);
    Handshake(PacketResult* result);

    string getCategory();
    int getErrorCode();

    bool isEqual(Handshake* other);
    uint32_t getPacketNum();
    bool shouldWriteAgain();
    bool didFail();

    unsigned int getPacket(char* buffer);

    ~Handshake();
};

class TunnelProtocol
{
private:
    vector<PacketResult*> _result_queue;
    int _read_buffer_index;
    uint32_t _read_packet_num;
    uint32_t _write_packet_num;
    int _current_segment_start;
    int _current_segment_stop;

    PacketResult* parsePacket(char* buffer, int start_index, int stop_index);
    bool parseNextSegment(char* buffer, int stop_index, char format, PacketResult* result);
    bool getNextSegment(char* buffer, int stop_index);
    bool getNextSegment(char* buffer, int stop_index, int length);

public:
    static const char PACKET_START_0 = 0x12;
    static const char PACKET_START_1 = 0x13;
    static const char PACKET_STOP = '\n';
    static const char PACKET_SEP = '\t';
    static const int MAX_PACKET_LEN = 1024;
    static const int MIN_PACKET_LEN = 13;
    static const int MAX_SEGMENT_LEN = 64;
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

    int parseBuffer(char* buffer, int start_index, int stop_index);
    PacketResult* popResult();
    bool isCodeError(int error_code);
    unsigned int makePacket(packet_type_t packet_type, char* write_buffer, string category, const char *formats, va_list args);
    uint32_t getWritePacketNum() { return _write_packet_num; }
    uint32_t getReadPacketNum() { return _read_packet_num; }
};
