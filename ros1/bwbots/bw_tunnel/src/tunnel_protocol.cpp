#include "tunnel_protocol.h"

uint64_union_t uint64_union_data;
uint64_t to_uint64(char* buffer)
{
    for (unsigned short i = 0; i < sizeof(uint64_t); i++) {
        uint64_union_data.byte[sizeof(uint64_t) - i - 1] = buffer[i];
    }
    return uint64_union_data.integer;
}

int64_union_t int64_union_data;
int64_t to_int64(char* buffer)
{
    for (unsigned short i = 0; i < sizeof(int64_t); i++) {
        int64_union_data.byte[sizeof(int64_t) - i - 1] = buffer[i];
    }
    return int64_union_data.integer;
}

uint32_union_t uint32_union_data;
uint32_t to_uint32(char* buffer)
{
    for (unsigned short i = 0; i < sizeof(uint32_t); i++) {
        uint32_union_data.byte[sizeof(uint32_t) - i - 1] = buffer[i];
    }
    return uint32_union_data.integer;
}

int32_union_t int32_union_data;
int32_t to_int32(char* buffer)
{
    for (unsigned short i = 0; i < sizeof(int32_t); i++) {
        int32_union_data.byte[sizeof(int32_t) - i - 1] = buffer[i];
    }
    return int32_union_data.integer;
}

uint16_union_t uint16_union_data;
uint16_t to_uint16(char* buffer)
{
    for (unsigned short i = 0; i < sizeof(uint16_t); i++) {
        uint16_union_data.byte[sizeof(uint16_t) - i - 1] = buffer[i];
    }
    return uint16_union_data.integer;
}

int16_union_t int16_union_data;
int16_t to_int16(char* buffer)
{
    for (unsigned short i = 0; i < sizeof(int16_t); i++) {
        int16_union_data.byte[sizeof(int16_t) - i - 1] = buffer[i];
    }
    return int16_union_data.integer;
}

float to_float(char* buffer)
{
    float_union_t float_union_data;
    for (unsigned short i = 0; i < sizeof(float); i++) {
        float_union_data.byte[i] = buffer[i];
    }
    return float_union_data.floating_point;
}

double to_double(char* buffer)
{
    double_union_t double_union_data;
    for (unsigned short i = 0; i < sizeof(double); i++) {
        double_union_data.byte[i] = buffer[i];
    }
    return double_union_data.floating_point;
}


string to_string(char* buffer, int length)
{
    if (length < 0) {
        ROS_ERROR("Can't convert string if length is less than zero");
        return "";
    }
    char char_array[length + 1];
    memcpy(char_array, buffer, length);
    char_array[length] = '\0';
    return string(char_array);
}

char RECV_CHECKSUM_ARRAY[3];

uint8_t from_checksum(char* buffer)
{
    char recv_checksum_array[3];
    memcpy(recv_checksum_array, buffer, 2);
    recv_checksum_array[2] = '\0';
    return strtol(recv_checksum_array, NULL, 16);
}

string format_char(unsigned char c)
{
    if (c == 92) return "\\\\";
    else if (c == 9) return "\\t";
    else if (c == 10) return "\\n";
    else if (c == 13) return "\\r";
    else if (c == 11 || c == 12 || c <= 9 || (14 <= c && c <= 31) || 127 <= c)
    {
        char* temp_buf = new char[6];
        sprintf(temp_buf, "\\x%02x", c);
        return string(temp_buf);
    }
    else {
        return string(1, (char)c);
    }
}

string packetToString(char* buffer, int start_index, int stop_index)
{
    string str = "";
    for (size_t i = start_index; i < stop_index; i++) {
        str += format_char(buffer[i]);
    }
    return str;
}

// ---
// PacketResult
// ---

bool PacketResult::checkIndex() {
    if (_current_index >= _stop_index) {
        ROS_WARN("Index exceeds buffer limits. %d >= %d", _current_index, _stop_index);
        return false;
    }
    return true;
}
PacketResult::PacketResult(PacketResult* result)
{
    setFrom(result);
}
PacketResult::PacketResult(int error_code, ros::Time recv_time) {
    _category = "";
    _recv_time = recv_time;
    _error_code = error_code;
    _packet_type = PACKET_TYPE_NORMAL;
    _packet_num = 0;
    _buffer = NULL;
}

PacketResult::~PacketResult() {
    // if (_buffer != NULL) {
    //     free(_buffer);
    // }
}
void PacketResult::setFrom(PacketResult* result) {
    _category = result->getCategory();
    _recv_time = result->getRecvTime();
    _error_code = result->getErrorCode();
    _packet_type = result->getPacketType();
    _packet_num = result->getPacketNum();
    _start_index = 0;
    _stop_index = result->getStop() - result->getStart();
    if (_stop_index < 0) {
        _stop_index = 0;
    }
    else {
        _buffer = (char*)malloc(sizeof(char) * _stop_index);
        memcpy(_buffer, result->getBuffer(), sizeof(char) * _stop_index);
    }
}

void PacketResult::setCategory(string category) {
    _category = category;
}
string PacketResult::getCategory() {
    return _category;
}
void PacketResult::setPacketType(packet_type_t packet_type) {
    _packet_type = packet_type;
}
packet_type_t PacketResult::getPacketType() {
    return _packet_type;
}
void PacketResult::setPacketNum(uint32_t packet_num) {
    _packet_num = packet_num;
}
uint32_t PacketResult::getPacketNum() {
    return _packet_num;
}
void PacketResult::setErrorCode(int error_code) {
    _error_code = error_code;
}
int PacketResult::getErrorCode() {
    return _error_code;
}
void PacketResult::setRecvTime(ros::Time recv_time) {
    _recv_time = recv_time;
}
ros::Time PacketResult::getRecvTime() {
    return _recv_time;
}
void PacketResult::setBuffer(char* buffer) {
    _buffer = buffer;
}
char* PacketResult::getBuffer() {
    return _buffer;
}
void PacketResult::setStart(int index) {
    _start_index = index;
    _current_index = _start_index;
}
int PacketResult::getStart() {
    return _start_index;
}
void PacketResult::setStop(int index) {
    _stop_index = index;
}
int PacketResult::getStop() {
    return _stop_index;
}
bool PacketResult::getInt64(int64_t& result) {
    if (_buffer == NULL) { return false; }
    result = to_int64(_buffer + _current_index);
    _current_index += sizeof(int64_t);
    return checkIndex();
}
bool PacketResult::getUInt64(uint64_t& result) {
    if (_buffer == NULL) { return false; }
    result = to_uint64(_buffer + _current_index);
    _current_index += sizeof(uint64_t);
    return checkIndex();
}
bool PacketResult::getInt32(int32_t& result) {
    if (_buffer == NULL) { return false; }
    result = to_int32(_buffer + _current_index);
    _current_index += sizeof(int32_t);
    return checkIndex();
}
bool PacketResult::getUInt32(uint32_t& result) {
    if (_buffer == NULL) { return false; }
    result = to_uint32(_buffer + _current_index);
    _current_index += sizeof(uint32_t);
    return checkIndex();
}
bool PacketResult::getInt16(int16_t& result) {
    if (_buffer == NULL) { return false; }
    result = to_int16(_buffer + _current_index);
    _current_index += sizeof(int16_t);
    return checkIndex();
}
bool PacketResult::getUInt16(uint16_t& result) {
    if (_buffer == NULL) { return false; }
    result = to_uint16(_buffer + _current_index);
    _current_index += sizeof(uint16_t);
    return checkIndex();
}
bool PacketResult::getInt8(int8_t& result) {
    if (_buffer == NULL) { return false; }
    result = (int8_t)_buffer[_current_index];
    _current_index += sizeof(int8_t);
    return checkIndex();
}
bool PacketResult::getUInt8(uint8_t& result) {
    if (_buffer == NULL) { return false; }
    result = (uint8_t)_buffer[_current_index];
    _current_index += sizeof(uint8_t);
    return checkIndex();
}
bool PacketResult::getBool(bool& result) {
    if (_buffer == NULL) { return false; }
    result = (bool)_buffer[_current_index];
    _current_index += sizeof(uint8_t);
    return checkIndex();
}
bool PacketResult::getFloat(float& result) {
    if (_buffer == NULL) { return false; }
    result = to_float(_buffer + _current_index);
    _current_index += sizeof(float);
    return checkIndex();
}
bool PacketResult::getDouble(double& result) {
    if (_buffer == NULL) { return false; }
    result = to_double(_buffer + _current_index);
    _current_index += sizeof(double);
    return checkIndex();
}
bool PacketResult::getString(string& result) {
    if (_buffer == NULL) { return false; }
    int length = to_uint16(_buffer + _current_index);
    _current_index += sizeof(uint16_t);
    getString(result, length);
    return checkIndex();
}
bool PacketResult::getString(string& result, int length) {
    if (_buffer == NULL) { return false; }
    result = to_string(_buffer + _current_index, length);
    _current_index += length;
    return checkIndex();
}

// ---
// Handshake
// ---


Handshake::Handshake(string category, char* buffer, unsigned int length, uint32_t packet_num, double write_interval, double timeout) {
    _category = category;
    _length = length;
    _write_interval = write_interval;
    _timeout = timeout;
    _error_code = TunnelProtocol::NULL_ERROR;
    _packet_num = packet_num;

    _packet = (char*)malloc(sizeof(char) * length);
    memcpy(_packet, buffer, sizeof(char) * length);
    _prev_write_time = ros::Time::now();
    _initial_write_time = _prev_write_time;
}
Handshake::Handshake(PacketResult* result) {
    int32_t packet_num;
    if (!result->getInt32(packet_num)) {
        packet_num = 0;
    }
    int32_t error_code;
    if (!result->getInt32(error_code)) {
        error_code = TunnelProtocol::NULL_ERROR;
    }
    _category = result->getCategory();
    _error_code = result->getErrorCode();
    _packet_num = (uint32_t)packet_num;
    _packet = NULL;
    _prev_write_time = ros::Time::now();
    _initial_write_time = _prev_write_time;
}

string Handshake::getCategory() {
    return _category;
}

int Handshake::getErrorCode() {
    return _error_code;
}

uint32_t Handshake::getPacketNum() {
    return _packet_num;
}

bool Handshake::isEqual(Handshake* other) {
    return _category == other->getCategory() && _packet_num == other->getPacketNum();
}

bool Handshake::shouldWriteAgain() {
    if (_write_interval <= 0.0) {
        return false;
    }
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - _prev_write_time).toSec();
    if (dt > _write_interval) {
        _prev_write_time = current_time;
        return true;
    }
    else {
        return false;
    }
}

bool Handshake::didFail() {
    if (_timeout > 0.0) {
        return (ros::Time::now() - _prev_write_time).toSec() > _timeout;
    }
    else {
        return false;
    }
}

unsigned int Handshake::getPacket(char* buffer)
{
    memcpy(buffer, _packet, sizeof(char) * _length);
    return _length;
}

Handshake::~Handshake() {
    if (_packet != NULL) {
        free(_packet);
    }
}

// ---
// TunnelProtocol
// ---

TunnelProtocol::TunnelProtocol()
{
    _read_packet_num = -1;
    _write_packet_num = 0;
    _read_buffer_index = 0;
}

TunnelProtocol::~TunnelProtocol()
{

}

unsigned int TunnelProtocol::makePacket(packet_type_t packet_type, char* write_buffer, string category, const char *formats, va_list args)
{
    unsigned int buffer_index = 0;
    write_buffer[buffer_index++] = PACKET_START_0;
    write_buffer[buffer_index++] = PACKET_START_1;
    buffer_index += 2;  // bytes 2 and 3 are for packet length which will be calculated later

    write_buffer[buffer_index++] = (uint8_t)packet_type;

    uint32_union_t union_packet_num;
    union_packet_num.integer = _write_packet_num;
    for (unsigned short i = 0; i < 4; i++) {
        write_buffer[buffer_index++] = union_packet_num.byte[3 - i];
    }
    sprintf(write_buffer + buffer_index, "%s", category.c_str());
    buffer_index += category.length();
    write_buffer[buffer_index++] = '\t';

    while (*formats != '\0')
    {
        if (*formats == 'd') {  // 32 bit signed
            int32_union_t int32_union_data;
            int32_union_data.integer = va_arg(args, int);
            for (unsigned short i = 0; i < sizeof(int32_t); i++) {
                write_buffer[buffer_index++] = int32_union_data.byte[sizeof(int32_t) - i - 1];
            }
        }
        else if (*formats == 'u') {  // 32 bit unsigned
            uint32_union_t uint32_union_data;
            uint32_union_data.integer = va_arg(args, int);
            for (unsigned short i = 0; i < sizeof(uint32_t); i++) {
                write_buffer[buffer_index++] = uint32_union_data.byte[sizeof(uint32_t) - i - 1];
            }
        }
        else if (*formats == 'l') {  // 64 bit signed
            int64_union_t int64_union_data;
            int64_union_data.integer = va_arg(args, int64_t);
            for (unsigned short i = 0; i < sizeof(int64_t); i++) {
                write_buffer[buffer_index++] = int64_union_data.byte[sizeof(int64_t) - i - 1];
            }
        }
        else if (*formats == 'm') {  // 64 bit unsigned
            uint64_union_t uint64_union_data;
            uint64_union_data.integer = va_arg(args, uint64_t);
            for (unsigned short i = 0; i < sizeof(uint64_t); i++) {
                write_buffer[buffer_index++] = uint64_union_data.byte[sizeof(uint64_t) - i - 1];
            }
        }
        else if (*formats == 'b') {  // 8 bit signed
            int8_t value = va_arg(args, int);
            write_buffer[buffer_index++] = value;
        }
        else if (*formats == 'c') {  // 8 bit unsigned
            uint8_t value = va_arg(args, int);
            write_buffer[buffer_index++] = value;
        }
        else if (*formats == 'h') {  // 16 bit signed
            int16_union_t int16_union_data;
            int16_union_data.integer = va_arg(args, int);
            for (unsigned short i = 0; i < sizeof(int16_t); i++) {
                write_buffer[buffer_index++] = int16_union_data.byte[sizeof(int16_t) - i - 1];
            }
        }
        else if (*formats == 'g') {  // 16 bit unsigned
            uint16_union_t uint16_union_data;
            uint16_union_data.integer = va_arg(args, int);
            for (unsigned short i = 0; i < sizeof(uint16_t); i++) {
                write_buffer[buffer_index++] = uint16_union_data.byte[sizeof(uint16_t) - i - 1];
            }
        }
        else if (*formats == 's') {
            char *s = va_arg(args, char*);
            uint16_union_t uint16_union_data;
            uint16_union_data.integer = (uint16_t)strlen(s);
            for (unsigned short i = 0; i < sizeof(uint16_t); i++) {
                write_buffer[buffer_index++] = uint16_union_data.byte[sizeof(uint16_t) - i - 1];
            }
            sprintf(write_buffer + buffer_index, "%s", s);
            buffer_index += strlen(s);
        }
        else if (*formats == 'x') {
            char *s = va_arg(args, char*);
            uint16_union_t uint16_union_data;
            uint16_union_data.byte[1] = s[0];
            uint16_union_data.byte[0] = s[1];
            if (uint16_union_data.integer > MAX_SEGMENT_LEN) {
                ROS_ERROR("Packet segment is too long. Category: %s", category.c_str());
                va_end(args);
                return -1;
            }
            write_buffer[buffer_index++] = s[0];
            write_buffer[buffer_index++] = s[1];
            for (size_t i = 0; i < uint16_union_data.integer; i++) {
                write_buffer[buffer_index++] = s[2 + i];
            }
        }
        else if (*formats == 'f') {
            float_union_t float_union_data;
            float_union_data.floating_point = (float)va_arg(args, double);  // va_arg promotes floats to doubles
            for (unsigned short i = 0; i < sizeof(float); i++) {
                write_buffer[buffer_index++] = float_union_data.byte[i];
            }
        }
        else if (*formats == 'e') {
            double_union_t double_union_data;
            double_union_data.floating_point = (double)va_arg(args, double);
            for (unsigned short i = 0; i < sizeof(double); i++) {
                write_buffer[buffer_index++] = double_union_data.byte[i];
            }
        }
        else {
            ROS_ERROR("Invalid format type. Category: %s", category.c_str());
            va_end(args);
            return -1;
        }
        ++formats;
    }
    uint8_t calc_checksum = 0;
    for (size_t index = 4; index < buffer_index; index++) {
        calc_checksum += (uint8_t)write_buffer[index];
    }

    sprintf(write_buffer + buffer_index, "%02x", calc_checksum);
    buffer_index += 2;
    write_buffer[buffer_index++] = '\n';
    write_buffer[buffer_index] = '\0';

    uint16_t packet_len = buffer_index - 5;  // subtract start, length, and stop bytes

    // insert packet length
    uint16_union_t union_packet_len;
    union_packet_len.integer = packet_len;
    write_buffer[2] = union_packet_len.byte[1];
    write_buffer[3] = union_packet_len.byte[0];

    _write_packet_num++;

    return buffer_index;
}


int TunnelProtocol::parseBuffer(char* buffer, int start_index, int stop_index)
{
    int last_packet_index = 0;
    int index;
    for (index = start_index; index < stop_index; index++) {
        int packet_start = index;
        if (buffer[index] != PACKET_START_0) {
            // ROS_DEBUG("Index %d is not PACKET_START_0", index);
            continue;
        }
        index++;
        if (index >= stop_index) {
            index = stop_index;
            continue;
        }
        if (buffer[index] != PACKET_START_1) {
            // ROS_DEBUG("Index %d is not PACKET_START_1", index);
            continue;
        }
        index++;
        if (index >= stop_index) {
            index = stop_index;
            continue;
        }

        uint16_t length = to_uint16(buffer + index);
        index += 2;

        if (index >= stop_index) {
            ROS_DEBUG("Buffer length exceeded while searching for length start");
            index = stop_index;
            continue;
        }
        
        // ROS_DEBUG("Found packet length: %d", length);
        index += length;

        if (index > stop_index) {
            ROS_DEBUG("Buffer length exceeded while searching for length stop. %d > %d", index, stop_index);
            index = stop_index;
            continue;
        }
        if (buffer[index] != PACKET_STOP) {
            ROS_DEBUG("Buffer does not end with PACKET_STOP");
            continue;
        }
        // do not modify index from this point onward as the for loop increments index
        last_packet_index = index + 1;
        // ROS_INFO("Found a packet: %s. %d..%d", packetToString(buffer, packet_start, index).c_str(), packet_start, index);
        PacketResult* result = parsePacket(buffer, packet_start, last_packet_index);
        _result_queue.push_back(result);
    }

    return last_packet_index;
}

PacketResult* TunnelProtocol::popResult()
{
    if (_result_queue.size() == 0) {
        return new PacketResult(NULL_ERROR, ros::Time::now());
    }
    PacketResult* result = _result_queue.at(0);
    _result_queue.erase(_result_queue.begin());
    return result;
}

bool TunnelProtocol::isCodeError(int error_code)
{
    switch (error_code) {
        case NO_ERROR:
        case PACKET_COUNT_NOT_SYNCED_ERROR:
        case NULL_ERROR:
            return false;
        default:
            return true;
    }
}

PacketResult* TunnelProtocol::parsePacket(char* buffer, int start_index, int stop_index)
{
    _read_buffer_index = start_index;
    ros::Time recv_time = ros::Time::now();
    int length = stop_index - start_index;
    PacketResult* result = new PacketResult(NULL_ERROR, recv_time);
    if (length < MIN_PACKET_LEN) {
        ROS_INFO("Packet is not the minimum length (%d)", MIN_PACKET_LEN);
        result->setErrorCode(PACKET_TOO_SHORT_ERROR);
        return result;
    }

    if (buffer[_read_buffer_index] != PACKET_START_0) {
        ROS_INFO("Packet does not start with PACKET_START_0");
        _read_packet_num++;
        result->setErrorCode(PACKET_0_ERROR);
        return result;
    }
    _read_buffer_index++;
    if (buffer[_read_buffer_index] != PACKET_START_1) {
        ROS_INFO("Packet does not start with PACKET_START_1");
        _read_packet_num++;
        result->setErrorCode(PACKET_1_ERROR);
        return result;
    }
    _read_buffer_index++;
    if (buffer[stop_index - 1] != PACKET_STOP) {
        ROS_INFO("Packet does not start with PACKET_STOP");
        _read_packet_num++;
        result->setErrorCode(PACKET_STOP_ERROR);
        return result;
    }

    int checksum_start = stop_index - 3;

    _read_buffer_index += LENGTH_BYTE_LENGTH;
    uint8_t calc_checksum = 0;
    // compute checksum using all characters except the checksum itself
    for (int index = _read_buffer_index; index < checksum_start; index++) {
        calc_checksum += (uint8_t)buffer[index];
    }

    uint8_t recv_checksum = from_checksum(buffer + checksum_start);

    if (calc_checksum != recv_checksum) {
        ROS_INFO("Checksum failed! recv %02x != calc %02x", recv_checksum, calc_checksum);
        _read_packet_num++;
        result->setErrorCode(CHECKSUMS_DONT_MATCH_ERROR);
        return result;
    }

    if (!getNextSegment(buffer, stop_index, 1)) {
        ROS_WARN("Failed to find packet type segment!");
        _read_packet_num++;
        result->setErrorCode(PACKET_TYPE_NOT_FOUND_ERROR);
        return result;
    }

    packet_type_t packet_type = (packet_type_t)(uint8_t)(buffer[_current_segment_start]);
    result->setPacketType(packet_type);

    if (!getNextSegment(buffer, stop_index, 4)) {
        ROS_WARN("Failed to find packet number segment!");
        _read_packet_num++;
        result->setErrorCode(PACKET_COUNT_NOT_FOUND_ERROR);
        return result;
    }

    uint32_t recv_packet_num = to_uint32(buffer + _current_segment_start);

    if (_read_packet_num == -1) {
        _read_packet_num = recv_packet_num;
    }

    result->setErrorCode(NO_ERROR);


    if (packet_type == PACKET_TYPE_HANDSHAKE) {
        result->setPacketNum(recv_packet_num);
    }
    else if (recv_packet_num != _read_packet_num) {
        ROS_WARN("Received packet num doesn't match local count. recv %d != local %d", recv_packet_num, _read_packet_num);
        _read_packet_num++;
        _read_packet_num = recv_packet_num;
        result->setErrorCode(PACKET_COUNT_NOT_SYNCED_ERROR);
    }

    if (!getNextSegment(buffer, stop_index)) {
        ROS_WARN("Failed to find category segment!");
        _read_packet_num++;
        result->setErrorCode(PACKET_CATEGORY_ERROR);
        return result;
    }

    string category = to_string(buffer + _current_segment_start, _current_segment_stop - _current_segment_start);
    if (category.size() == 0) {
        ROS_WARN("Category segment is empty");
        _read_packet_num++;
        result->setErrorCode(PACKET_CATEGORY_ERROR);
        return result;
    }
    result->setCategory(category);

    // _read_buffer_index is currently the next index after category separator (\t)
    result->setStart(_read_buffer_index);
    result->setStop(checksum_start + 1);

    result->setBuffer(buffer);
    result->setErrorCode(NO_ERROR);
    _read_packet_num++;
    // ROS_DEBUG("Parsed packet: %s", packetToString(buffer, start_index, stop_index).c_str());
    
    return result;
}

bool TunnelProtocol::getNextSegment(char* buffer, int stop_index, int length)
{
    if (_read_buffer_index >= stop_index + length) {
        return false;
    }
    if (length == -1) {
        length = to_uint16(buffer + _read_buffer_index);
        _read_buffer_index += 2;
        if (length >= stop_index + length) {
            ROS_ERROR("Parsed length %d exceeds buffer length! %d", length, _read_buffer_index + length);
            return false;
        }
    }
    _current_segment_start = _read_buffer_index;
    _read_buffer_index += length;
    _current_segment_stop = _read_buffer_index;
    return true;
}

bool TunnelProtocol::getNextSegment(char* buffer, int stop_index)
{
    if (_read_buffer_index >= stop_index) {
        return false;
    }
    int sep_index;
    for (sep_index = _read_buffer_index; sep_index < stop_index; sep_index++) {
        if (buffer[sep_index] == PACKET_SEP) {
            break;
        }
    }
    if (sep_index >= stop_index) {
        _current_segment_start = _read_buffer_index;
        _current_segment_stop = stop_index;
        _read_buffer_index = stop_index;
    }
    else {
        _current_segment_start = _read_buffer_index;
        _current_segment_stop = sep_index;
        _read_buffer_index = sep_index + 1;
    }
    return true;
}
