#pragma once

#include <Arduino.h>
#include "tunnel/protocol.h"

#define PROTOCOL_SERIAL Serial5
#define PROTOCOL_BAUD 1000000

#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL PROTOCOL_SERIAL
#endif


const uint32_t PACKET_STOP_TIMEOUT = 500;

void tunnel_begin();
PacketResult* tunnel_readPacket();
void tunnel_writePacket(const char *category, const char *formats, ...);
void tunnel_writeConfirmingPacket(const char *category, const char *formats, ...);
void tunnel_writeBuffer(int length);
