#ifndef RS485_DRIVER_H_
#define RS485_DRIVER_H_
#include "board.h"

uint8_t rs485_transmit(const dataPacket * const packet);
uint8_t rs485_receive(dataPacket *packet);

#endif