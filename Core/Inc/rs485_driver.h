#ifndef RS485_DRIVER_H_
#define RS485_DRIVER_H_

typedef struct{
  uint8_t data[256];
  uint8_t data_len;
} modbusPacket;

uint8_t rs485_transmit(const modbusPacket * const packet);
uint8_t rs485_receive(modbusPacket *packet);

#endif